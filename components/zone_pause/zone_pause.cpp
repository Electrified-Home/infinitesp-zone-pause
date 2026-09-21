#include "zone_pause.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace zone_pause {

static const char *const TAG = "zone_pause";
static const uint8_t SAVED_VERSION = 1;
// A result is judged only after this long. The hub sends a write up to 3 times,
// 4 s apart, so the last attempt can go out about 8 s after the request (later for
// the second of two queued writes). The thermostat then needs a few seconds to
// adopt it, and it is polled every 3 s with roughly 1 poll in 8 unanswered.
static const uint32_t SETTLE_MS = 30000;
static const uint8_t MAX_ATTEMPTS = 2;
static const uint32_t TICK_INTERVAL_MS = 1000;
// Carrier's heat/cool deadband. An edited target that violates it is rejected
// here because the hub's write path does not check it.
static const uint8_t MIN_GAP = 2;
// A paused zone whose hold stops being permanent for this long was taken over by
// something else. The wait rides out the odd inconsistent poll.
static const uint32_t HOLD_LOST_MS = 15000;

static const uint16_t HOLD_PERMANENT = infinitesp::InfinitESPComponent::HOLD_PERMANENT;

void ZonePauseSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->request_pause(state);
}

void ZonePauseClimate::init() {
  this->set_supported_custom_presets({
      infinitesp::PRESET_SCHEDULE,  infinitesp::PRESET_WAKE,     infinitesp::PRESET_HOLD_TIMED,
      infinitesp::PRESET_HOLD_PERM, infinitesp::PRESET_VACATION,
  });
  this->data_ = Saved{};
  this->data_.version = SAVED_VERSION;
  this->source_->add_on_state_callback([this](climate::Climate &) {
    if (!this->started_)
      return;
    this->evaluate_();
    this->publish_all_();
  });
}

// Runs on the first bus notification, when preferences and the hub are fully up.
// In active (SAM) mode the hub sends one during its own setup, so a saved pause is back
// before Home Assistant connects. With sam_address 0 nothing here ever starts; pause
// cannot work without SAM mode anyway, because it has no way to write.
void ZonePauseClimate::ensure_started_() {
  if (this->started_)
    return;
  this->started_ = true;
  this->pref_ = this->make_entity_preference<Saved>(0x5A4F4E45UL);
  Saved loaded{};
  if (this->pref_.load(&loaded) && loaded.version == SAVED_VERSION && loaded.paused) {
    this->data_ = loaded;
    this->needs_reconcile_ = true;
    this->restarted_since_pause_ = true;
    ESP_LOGI(TAG, "Zone %d was paused before the restart; checking it against the thermostat",
             this->source_->get_zone());
  }
  ESP_LOGI(TAG, "Zone %d: wide setpoints %d / %d F, paused: %s", this->source_->get_zone(), this->pause_heat_f_,
           this->pause_cool_f_, YESNO(this->data_.paused));
}

void ZonePauseClimate::on_register_update(uint8_t device_addr, uint16_t register_key) {
  // Used purely as a heartbeat; the hub notifies on every register it stores.
  const bool first = !this->started_;
  this->ensure_started_();
  if (first)
    this->publish_all_();
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->last_tick_ms_;
  if (elapsed < TICK_INTERVAL_MS)
    return;
  this->last_tick_ms_ = now;
  if (this->data_.paused) {
    this->minute_accum_ms_ += elapsed;
    while (this->minute_accum_ms_ >= 60000) {
      this->minute_accum_ms_ -= 60000;
      if (this->paused_minutes_ < 0xFFFF)
        this->paused_minutes_++;
    }
  }
  this->evaluate_();
}

climate::ClimateTraits ZonePauseClimate::traits() {
  // Same capabilities as the InfinitESP zone it fronts. Its custom presets are registered
  // on the entity in init(). Pause is not a preset: the pause switch is the only place
  // that says whether a zone is paused.
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION |
                           climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE);
  static const float F_TO_C = 5.0f / 9.0f;
  traits.set_visual_min_temperature((40.0f - 32.0f) * F_TO_C);
  traits.set_visual_max_temperature((99.0f - 32.0f) * F_TO_C);
  traits.set_visual_temperature_step(1.0f);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  traits.add_supported_preset(climate::CLIMATE_PRESET_HOME);
  traits.add_supported_preset(climate::CLIMATE_PRESET_AWAY);
  traits.add_supported_preset(climate::CLIMATE_PRESET_SLEEP);
  return traits;
}

bool ZonePauseClimate::bus_ready_() const {
  return this->parent_->is_bus_online() && this->parent_->has_real_state();
}

uint8_t ZonePauseClimate::pause_heat_bus_() const {
  return this->parent_->celsius_to_setpoint((this->pause_heat_f_ - 32.0f) * 5.0f / 9.0f);
}
uint8_t ZonePauseClimate::pause_cool_bus_() const {
  return this->parent_->celsius_to_setpoint((this->pause_cool_f_ - 32.0f) * 5.0f / 9.0f);
}

// Reads the zone's setpoints from the thermostat's own last reply. The hub stores
// every reply under the sender's address (0x20 here), and only real replies land
// there. The copy under the SAM address is not used: the hub also writes that one
// optimistically the moment a command is queued.
bool ZonePauseClimate::read_actual_(uint8_t &heat, uint8_t &cool) const {
  const auto *zones = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT, infinitesp::REG_SAM_ZONES);
  if (zones == nullptr || zones->size() < infinitesp::REG3B03_SIZE)
    return false;
  const uint8_t idx = this->source_->get_zone() - 1;
  heat = (*zones)[infinitesp::REG3B03_HEAT_SETPOINTS + idx];
  cool = (*zones)[infinitesp::REG3B03_COOL_SETPOINTS + idx];
  return heat != 0 && cool != 0;
}

void ZonePauseClimate::control(const climate::ClimateCall &call) {
  this->ensure_started_();
  auto fwd = this->source_->make_call();
  bool forward = false;

  // Mode and fan are not affected by pause. They always go straight through.
  if (call.get_mode().has_value()) {
    fwd.set_mode(*call.get_mode());
    forward = true;
  }
  if (call.get_fan_mode().has_value()) {
    fwd.set_fan_mode(*call.get_fan_mode());
    forward = true;
  }

  optional<float> low = call.get_target_temperature_low();
  optional<float> high = call.get_target_temperature_high();
  if (call.get_target_temperature().has_value()) {
    // Judge by the mode this same call asks for, if any, like InfinitESP does.
    const climate::ClimateMode mode = call.get_mode().value_or(this->mode);
    if (mode == climate::CLIMATE_MODE_HEAT)
      low = call.get_target_temperature();
    else if (mode == climate::CLIMATE_MODE_COOL)
      high = call.get_target_temperature();
  }
  if (low.has_value() || high.has_value()) {
    if (this->data_.paused) {
      uint8_t heat = low.has_value() ? this->parent_->celsius_to_setpoint(*low) : this->data_.target_heat;
      uint8_t cool = high.has_value() ? this->parent_->celsius_to_setpoint(*high) : this->data_.target_cool;
      if (heat + MIN_GAP > cool) {
        ESP_LOGW(TAG, "Zone %d: target %d / %d rejected, heat and cool are too close", this->source_->get_zone(), heat,
                 cool);
      } else {
        this->data_.target_heat = heat;
        this->data_.target_cool = cool;
        this->data_.target_changed = true;
        this->save_();
        ESP_LOGI(TAG, "Zone %d paused: target now %d / %d, applied on resume", this->source_->get_zone(), heat, cool);
      }
    } else {
      if (low.has_value())
        fwd.set_target_temperature_low(*low);
      if (high.has_value())
        fwd.set_target_temperature_high(*high);
      forward = true;
      if (this->phase_ == PHASE_RESUMING) {
        if (low.has_value())
          this->data_.target_heat = this->parent_->celsius_to_setpoint(*low);
        if (high.has_value())
          this->data_.target_cool = this->parent_->celsius_to_setpoint(*high);
      }
    }
  }

  if (call.has_custom_preset()) {
    auto custom = call.get_custom_preset();
    if (this->data_.paused) {
      ESP_LOGW(TAG, "Zone %d: preset change ignored while paused", this->source_->get_zone());
    } else {
      fwd.set_preset(custom.c_str(), custom.size());
      forward = true;
    }
  }
  if (call.get_preset().has_value()) {
    if (this->data_.paused) {
      ESP_LOGW(TAG, "Zone %d: preset change ignored while paused", this->source_->get_zone());
    } else {
      fwd.set_preset(*call.get_preset());
      forward = true;
    }
  }

  // The source publishes at the end of every call it handles, and its state callback
  // already republishes this entity, so publish here only when nothing was forwarded.
  if (forward)
    fwd.perform();
  else
    this->publish_all_();
}

void ZonePauseClimate::request_pause(bool pause) {
  this->ensure_started_();
  const uint8_t zone = this->source_->get_zone();

  if (this->phase_ != PHASE_IDLE || this->needs_reconcile_) {
    // A pause or resume is still settling. Remember the latest request and act on
    // it when that finishes, so a flapping door cannot leave the zone in the wrong state.
    this->queued_request_valid_ = true;
    this->queued_request_ = pause;
    ESP_LOGI(TAG, "Zone %d: %s requested while busy, queued", zone, pause ? "pause" : "resume");
    this->publish_all_();
    return;
  }

  // Pausing a paused zone keeps the original remembered settings (it must never
  // remember the wide values); resuming a running zone does nothing.
  if (pause == this->data_.paused) {
    this->publish_all_();
    return;
  }

  if (pause) {
    uint8_t heat = 0, cool = 0;
    if (!this->bus_ready_() || !this->read_actual_(heat, cool)) {
      ESP_LOGW(TAG, "Zone %d: cannot pause, no thermostat data yet", zone);
      this->publish_all_();
      return;
    }
    this->pre_heat_ = heat;
    this->pre_cool_ = cool;
    this->data_.target_heat = heat;
    this->data_.target_cool = cool;
    this->data_.hold_minutes = this->parent_->get_zone_hold_duration(zone);
    this->data_.target_changed = false;
    this->data_.applied_heat = this->pause_heat_bus_();
    this->data_.applied_cool = this->pause_cool_bus_();
    this->data_.paused = true;
    this->restarted_since_pause_ = false;
    this->hold_lost_ = false;
    this->paused_minutes_ = 0;
    this->minute_accum_ms_ = 0;
    ESP_LOGI(TAG, "Zone %d: PAUSE. Remembering %d / %d (hold %u), holding wide at %d / %d", zone, heat, cool,
             this->data_.hold_minutes, this->data_.applied_heat, this->data_.applied_cool);
    this->phase_attempts_ = 0;
    this->send_pause_writes_();
    this->start_phase_(PHASE_PAUSING);
  } else {
    ESP_LOGI(TAG, "Zone %d: RESUME to %d / %d (hold %u, target edited: %s)", zone, this->data_.target_heat,
             this->data_.target_cool, this->data_.hold_minutes, YESNO(this->data_.target_changed));
    this->data_.paused = false;
    this->phase_attempts_ = 0;
    this->send_resume_writes_();
    this->start_phase_(PHASE_RESUMING);
  }
  this->save_();
  this->publish_all_();
}

// In both write helpers the hub calls run back to back in one call stack. Each
// hub call updates the hub's working copy of the zones register before returning,
// so the second command is built on top of the first. Do not split them across
// loop iterations: a thermostat reply arriving in between would reset that copy.

void ZonePauseClimate::send_pause_writes_() {
  const uint8_t zone = this->source_->get_zone();
  this->parent_->set_zone_setpoint(zone, this->pause_heat_bus_(), this->pause_cool_bus_());
  this->parent_->set_zone_hold(zone, HOLD_PERMANENT);
}

void ZonePauseClimate::send_resume_writes_() {
  const uint8_t zone = this->source_->get_zone();
  const uint16_t hold = this->data_.hold_minutes;
  if (this->data_.target_changed) {
    // Behave like an ordinary setpoint change: leave the pause hold first, then set
    // the temperatures and let the thermostat start its usual hold.
    this->parent_->set_zone_hold(zone, 0);
    this->parent_->set_zone_setpoint(zone, this->data_.target_heat, this->data_.target_cool);
  } else if (hold == 0) {
    // Was following its schedule: go back to the schedule as it stands now.
    this->parent_->set_zone_hold(zone, 0);
  } else if (hold >= HOLD_PERMANENT) {
    // Was on a permanent hold. The pause hold is permanent too, so only the
    // setpoints need to go back.
    this->parent_->set_zone_setpoint(zone, this->data_.target_heat, this->data_.target_cool);
  } else if (!this->restarted_since_pause_ && hold > this->paused_minutes_ + 15) {
    this->parent_->set_zone_setpoint(zone, this->data_.target_heat, this->data_.target_cool);
    this->parent_->set_zone_hold(zone, hold - this->paused_minutes_);
  } else {
    // The timed hold it was on has run out (or the board restarted and lost track).
    this->parent_->set_zone_hold(zone, 0);
  }
}

void ZonePauseClimate::start_phase_(Phase phase) {
  this->phase_ = phase;
  this->phase_started_ms_ = millis();
  this->phase_attempts_++;
}

void ZonePauseClimate::end_pause_without_restore_(const char *reason) {
  ESP_LOGI(TAG, "Zone %d: pause ended, %s. Nothing restored.", this->source_->get_zone(), reason);
  this->data_.paused = false;
  this->hold_lost_ = false;
  this->phase_ = PHASE_IDLE;
  this->save_();
  this->publish_all_();
}

void ZonePauseClimate::evaluate_() {
  uint8_t heat = 0, cool = 0;
  const bool have = this->read_actual_(heat, cool);
  if (have)
    this->publish_actual_(heat, cool);
  const uint32_t now = millis();
  const uint8_t zone = this->source_->get_zone();
  const bool at_wide = have && heat == this->data_.applied_heat && cool == this->data_.applied_cool;
  // The hub refreshes its hold readback from thermostat replies every few seconds,
  // so outside the settle window after one of our own writes it reflects reality.
  const bool hold_permanent = this->parent_->get_zone_hold_duration(zone) >= HOLD_PERMANENT;

  if (this->needs_reconcile_) {
    if (!this->bus_ready_() || !have)
      return;
    this->needs_reconcile_ = false;
    if (!at_wide) {
      this->end_pause_without_restore_("the zone is not at the pause values after the restart");
    } else if (!hold_permanent) {
      // The board went down between the two pause commands. Finish the job.
      ESP_LOGW(TAG, "Zone %d: paused before the restart but the hold is not permanent, sending the pause again", zone);
      // The true pre-pause setpoints were lost with the restart. The saved target is the
      // closest stand-in; the settle check is dominated by the wide-values test anyway.
      this->pre_heat_ = this->data_.target_heat;
      this->pre_cool_ = this->data_.target_cool;
      this->phase_attempts_ = 0;
      this->send_pause_writes_();
      this->start_phase_(PHASE_PAUSING);
    } else {
      ESP_LOGI(TAG, "Zone %d: still paused after restart", zone);
      this->publish_all_();
    }
  } else if (this->phase_ == PHASE_PAUSING) {
    if (now - this->phase_started_ms_ < SETTLE_MS || !have)
      return;
    const bool untouched = heat == this->pre_heat_ && cool == this->pre_cool_;
    if (at_wide && hold_permanent) {
      this->phase_ = PHASE_IDLE;
      this->save_();
      ESP_LOGI(TAG, "Zone %d: pause confirmed at %d / %d, permanent hold", zone, heat, cool);
      this->publish_all_();
    } else if (at_wide || untouched) {
      // Still the old setpoints, or the wide ones without the permanent hold.
      if (this->phase_attempts_ < MAX_ATTEMPTS) {
        ESP_LOGW(TAG, "Zone %d: pause not fully adopted (setpoints %s, permanent hold %s), sending again", zone,
                 at_wide ? "ok" : "missing", hold_permanent ? "ok" : "missing");
        this->send_pause_writes_();
        this->start_phase_(PHASE_PAUSING);
        return;
      }
      if (at_wide) {
        // Never leave half a pause behind: wide setpoints on a hold that will run
        // out by itself. Put the zone back the way it was.
        ESP_LOGE(TAG, "Zone %d: thermostat did not take the permanent hold, undoing the pause", zone);
        this->data_.paused = false;
        this->phase_attempts_ = 0;
        this->send_resume_writes_();
        this->start_phase_(PHASE_RESUMING);
        this->save_();
        this->publish_all_();
      } else {
        ESP_LOGE(TAG, "Zone %d: thermostat did not accept the pause", zone);
        this->end_pause_without_restore_("the thermostat did not accept it");
      }
    } else {
      // Neither the old setpoints nor the wide ones: someone changed the zone while
      // the pause was taking effect (or the thermostat refuses these wide values and
      // picked its own). Either way the zone is not held where pause wants it.
      ESP_LOGW(TAG, "Zone %d: thermostat holds %d / %d, not the wide %d / %d. If this happens on every pause, the "
                    "configured wide values are outside what the thermostat accepts.",
               zone, heat, cool, this->data_.applied_heat, this->data_.applied_cool);
      this->end_pause_without_restore_("it was changed while the pause was taking effect");
    }
  } else if (this->phase_ == PHASE_RESUMING) {
    if (now - this->phase_started_ms_ < SETTLE_MS || !have)
      return;
    const bool target_is_wide =
        this->data_.target_heat == this->data_.applied_heat && this->data_.target_cool == this->data_.applied_cool;
    // Every resume except "it was on a permanent hold and nobody edited the target"
    // has to take the zone off the permanent hold that pausing put it on.
    const bool expect_permanent = !this->data_.target_changed && this->data_.hold_minutes >= HOLD_PERMANENT;
    const bool setpoints_pending = at_wide && !target_is_wide;
    const bool hold_pending = !expect_permanent && hold_permanent;
    if (setpoints_pending || hold_pending) {
      if (this->phase_attempts_ < MAX_ATTEMPTS) {
        ESP_LOGW(TAG, "Zone %d: resume not fully adopted (setpoints %s, pause hold %s), sending again", zone,
                 setpoints_pending ? "still wide" : "ok", hold_pending ? "still on" : "released");
        this->send_resume_writes_();
        this->start_phase_(PHASE_RESUMING);
        return;
      }
      this->phase_ = PHASE_IDLE;
      if (at_wide) {
        ESP_LOGE(TAG, "Zone %d: thermostat did not accept the resume, zone is still held wide", zone);
        this->data_.paused = true;
      } else {
        ESP_LOGE(TAG, "Zone %d: setpoints are back but the thermostat kept the permanent hold", zone);
      }
      this->save_();
    } else {
      this->phase_ = PHASE_IDLE;
      ESP_LOGI(TAG, "Zone %d: resume confirmed at %d / %d", zone, heat, cool);
    }
    this->publish_all_();
  } else if (this->data_.paused && have) {
    // Nothing this code did changes a paused zone, so any change came from the wall
    // control or something else that talks to the thermostat (the Carrier app,
    // Infinitude, vacation mode starting, the hub's own hold entities). That wins.
    if (!at_wide) {
      this->end_pause_without_restore_("it was changed at the thermostat");
    } else if (hold_permanent) {
      this->hold_lost_ = false;
    } else if (!this->hold_lost_) {
      this->hold_lost_ = true;
      this->hold_lost_since_ms_ = now;
    } else if (now - this->hold_lost_since_ms_ >= HOLD_LOST_MS) {
      this->end_pause_without_restore_("its hold was changed from somewhere else");
    }
  }

  if (this->phase_ == PHASE_IDLE && !this->needs_reconcile_ && this->queued_request_valid_) {
    this->queued_request_valid_ = false;
    this->request_pause(this->queued_request_);
  }
}

void ZonePauseClimate::mirror_from_source_() {
  this->current_temperature = this->source_->current_temperature;
  this->mode = this->source_->mode;
  this->action = this->source_->action;
  this->fan_mode = this->source_->fan_mode;

  if (this->data_.paused || this->phase_ == PHASE_RESUMING) {
    // Show the target, never the wide values. This also covers the settle window
    // after a resume, while the thermostat still reports the wide setpoints.
    this->target_temperature_low = this->parent_->setpoint_to_celsius(this->data_.target_heat);
    this->target_temperature_high = this->parent_->setpoint_to_celsius(this->data_.target_cool);
    this->clear_custom_preset_();
    this->preset.reset();
    return;
  }

  this->target_temperature_low = this->source_->target_temperature_low;
  this->target_temperature_high = this->source_->target_temperature_high;
  if (this->source_->has_custom_preset()) {
    this->set_custom_preset_(this->source_->get_custom_preset());
  } else if (this->source_->preset.has_value()) {
    this->set_preset_(*this->source_->preset);
  } else {
    this->clear_custom_preset_();
    this->preset.reset();
  }
}

void ZonePauseClimate::publish_actual_(uint8_t heat, uint8_t cool) {
  if (heat != this->last_actual_heat_) {
    this->last_actual_heat_ = heat;
    if (this->actual_heat_sensor_ != nullptr)
      this->actual_heat_sensor_->publish_state(this->parent_->setpoint_to_celsius(heat));
  }
  if (cool != this->last_actual_cool_) {
    this->last_actual_cool_ = cool;
    if (this->actual_cool_sensor_ != nullptr)
      this->actual_cool_sensor_->publish_state(this->parent_->setpoint_to_celsius(cool));
  }
}

void ZonePauseClimate::publish_all_() {
  this->mirror_from_source_();
  this->publish_state();
  if (this->pause_switch_ != nullptr &&
      (!this->switch_published_ || this->pause_switch_->state != this->data_.paused)) {
    this->switch_published_ = true;
    this->pause_switch_->publish_state(this->data_.paused);
  }
}

// save() alone only queues the data; ESPHome writes its queue to flash once a minute.
// A power cut inside that minute would forget a pause while the thermostat keeps the
// zone held wide, so write it through straight away. Unchanged data is skipped, and
// this runs a handful of times per pause, so flash wear is not a concern.
void ZonePauseClimate::save_() {
  if (this->pref_.save(&this->data_))
    global_preferences->sync();
}

}  // namespace zone_pause
}  // namespace esphome
