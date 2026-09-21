#include "zone_pause.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace zone_pause {

static const char *const TAG = "zone_pause";
static const uint8_t SAVED_VERSION = 2;
static const uint32_t SAVED_KEY = 0x5A4F4E46UL;
static const uint32_t TICK_INTERVAL_MS = 1000;
// At most one send per zone in this time. The hub repeats every write 3 times, 4 s apart,
// and its first-in first-out queue already puts the newest send last on the wire; this gap
// only keeps quick toggles from flipping the thermostat back and forth and crowding the bus.
static const uint32_t SEND_GAP_MS = 10000;
// Nothing is judged for this long after a send. The write is still being repeated (last
// attempt up to about 12 s after it was queued) and the thermostat's own reply was seen to
// lag a change by up to 15 s, so until then a real value can still be on its way to, or
// back from, a value of ours. Nothing visible waits for this.
static const uint32_t QUIET_MS = 30000;
// A value we sent can show up late (the gap, the quiet period, then some). After this long
// a real value equal to something we once sent is somebody else's doing.
static const uint32_t SENT_EXCUSE_MS = 60000;
// Sends repeated because a wanted value did not land, per goal and per mode change.
static const uint8_t MAX_RESENDS = 3;
// Real values older than this are not trusted for a snapshot or a send (polls come about
// every 6 s).
static const uint32_t FRESH_MS = 20000;
// Flash writes closer together than this are merged (a flapping switch).
static const uint32_t SYNC_GAP_MS = 2000;
// Carrier's usual heat/cool gap; climate.py checks the configured wide values the same way.
// An edited target that violates it is rejected here because the hub does not check it.
static const uint8_t MIN_GAP = 2;
// The thermostat ignores timed holds shorter than this (InfinitESP HOLD_TIMED_MIN).
static const uint16_t MIN_TIMED_HOLD = 15;
// When a change to a watched setpoint ends a pause, the other setpoint may still sit at
// its wide value. true = put back just the sides nobody touched.
static const bool RESTORE_UNTOUCHED_SIDES = true;

static const uint8_t MODE_UNKNOWN = 0xFF;
static const uint16_t HOLD_PERMANENT = infinitesp::InfinitESPComponent::HOLD_PERMANENT;
static const char *const SIDE_NAMES[2] = {"heat", "cool"};

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
  this->pref_ = this->make_entity_preference<Saved>(SAVED_KEY);
  Saved loaded{};
  if (this->pref_.load(&loaded)) {
    if (loaded.version != SAVED_VERSION) {
      // Accepted trade-off: a firmware update that changes this record forgets a pause.
      // Procedure: all pause switches off before installing an update.
      ESP_LOGW(TAG, "Zone %d: saved pause state is from another version and was ignored. If this zone was paused, "
                    "set its temperatures by hand.",
               this->source_->get_zone());
    } else if (loaded.paused || loaded.goal != GOAL_NONE) {
      this->data_ = loaded;
      this->needs_reconcile_ = true;
      this->restarted_since_pause_ = true;
      ESP_LOGI(TAG, "Zone %d: restarted while %s; checking against the thermostat", this->source_->get_zone(),
               loaded.paused ? "paused" : "settings were still being put back");
    }
  }
  ESP_LOGI(TAG, "Zone %d: wide setpoints %d / %d F, paused: %s", this->source_->get_zone(), this->pause_heat_f_,
           this->pause_cool_f_, YESNO(this->data_.paused));
}

void ZonePauseClimate::on_register_update(uint8_t device_addr, uint16_t register_key) {
  // Mostly a heartbeat (the hub notifies on every register it stores), and the one place
  // that sees when a real zones reply from the thermostat arrives.
  const uint32_t now = millis();
  if (device_addr == infinitesp::ADDR_THERMOSTAT && register_key == infinitesp::REG_SAM_ZONES) {
    this->have_zones_reply_ = true;
    this->last_zones_reply_ms_ = now;
  }
  const bool first = !this->started_;
  this->ensure_started_();
  if (first)
    this->publish_all_();
  const uint32_t elapsed = now - this->last_tick_ms_;
  if (elapsed < TICK_INTERVAL_MS)
    return;
  this->last_tick_ms_ = now;
  if (this->data_.goal != GOAL_NONE) {
    this->minute_accum_ms_ += elapsed;
    while (this->minute_accum_ms_ >= 60000) {
      this->minute_accum_ms_ -= 60000;
      if (this->paused_minutes_ < 0xFFFF)
        this->paused_minutes_++;
    }
  }
  if (this->dirty_)
    this->flush_();
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

// Real values: only from the thermostat's own last reply, which the hub stores under the
// sender's address (0x20). The hub's copy under the SAM address is never used here: the
// hub writes that one optimistically the moment a command is queued. The hold is decoded
// exactly as the hub decodes it (get_zone_hold_duration), only from the real copy.
bool ZonePauseClimate::read_real_(uint8_t real[2], bool &permanent, uint16_t *hold_minutes) const {
  const auto *zones = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT, infinitesp::REG_SAM_ZONES);
  if (zones == nullptr || zones->size() < infinitesp::REG3B03_SIZE)
    return false;
  const uint8_t idx = this->source_->get_zone() - 1;
  real[HEAT] = (*zones)[infinitesp::REG3B03_HEAT_SETPOINTS + idx];
  real[COOL] = (*zones)[infinitesp::REG3B03_COOL_SETPOINTS + idx];
  const bool holding = ((*zones)[infinitesp::REG3B03_ZONES_HOLDING] & (1 << idx)) != 0;
  const bool timed = ((*zones)[infinitesp::REG3B03_TIMED_HOLDS] & (1 << idx)) != 0;
  const uint16_t duration = ((uint16_t) (*zones)[infinitesp::REG3B03_HOLD_DURATIONS + idx * 2] << 8) |
                            (*zones)[infinitesp::REG3B03_HOLD_DURATIONS + idx * 2 + 1];
  permanent = holding && duration <= 1;
  if (hold_minutes != nullptr)
    *hold_minutes = permanent ? HOLD_PERMANENT : ((timed && duration == 0) ? 1 : duration);
  return real[HEAT] != 0 && real[COOL] != 0;
}

bool ZonePauseClimate::real_is_fresh_() const {
  return this->have_zones_reply_ && millis() - this->last_zones_reply_ms_ < FRESH_MS;
}

// The thermostat's own system mode. Not the source climate's mode, which is optimistic
// and flips between off and fan_only with the blower.
uint8_t ZonePauseClimate::read_confirmed_mode_() const {
  const auto *state = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT, infinitesp::REG_SAM_STATE);
  if (state == nullptr || state->size() <= infinitesp::REG3B02_STAGMODE)
    return MODE_UNKNOWN;
  return (*state)[infinitesp::REG3B02_STAGMODE] & 0x0F;
}

const char *ZonePauseClimate::mode_name_(uint8_t mode) {
  return mode < 6 ? infinitesp::SYSMODE_NAMES[mode] : "unknown";
}

// Which setpoint writes land over the SAM path in each system mode. HEAT takes heat only
// and OFF takes nothing (seen 2026-09-21); COOL and AUTO by symmetry, to be confirmed.
// One place to change when a test says otherwise.
void ZonePauseClimate::landing_sides_(uint8_t mode, bool &heat, bool &cool) {
  heat = cool = false;
  switch (mode) {
    case infinitesp::SYSMODE_HEAT:
    case infinitesp::SYSMODE_EHEAT:
    case infinitesp::SYSMODE_HEATPUMP:
      heat = true;
      break;
    case infinitesp::SYSMODE_COOL:
      cool = true;
      break;
    case infinitesp::SYSMODE_AUTO:
      heat = cool = true;
      break;
    default:  // off, unknown: nothing lands
      break;
  }
}

// Which setpoints end a pause when something else changes them, by design: in
// HEAT only heat, in COOL only cool, in AUTO and OFF both. A change to a side that is not
// watched is kept as that side's new target and the pause carries on.
void ZonePauseClimate::watched_sides_(uint8_t mode, bool &heat, bool &cool) {
  heat = cool = true;
  switch (mode) {
    case infinitesp::SYSMODE_HEAT:
    case infinitesp::SYSMODE_EHEAT:
    case infinitesp::SYSMODE_HEATPUMP:
      cool = false;
      break;
    case infinitesp::SYSMODE_COOL:
      heat = false;
      break;
    default:
      break;
  }
}

uint8_t ZonePauseClimate::goal_value_(Side side) const {
  if (this->data_.goal == GOAL_PAUSE)
    return side == HEAT ? this->data_.wide_heat : this->data_.wide_cool;
  return side == HEAT ? this->data_.target_heat : this->data_.target_cool;
}

void ZonePauseClimate::adopt_target_(Side side, uint8_t value) {
  if (side == HEAT)
    this->data_.target_heat = value;
  else
    this->data_.target_cool = value;
}

// The hold a restore puts back. A timed hold comes back only with more than the
// thermostat's minimum left, and never after a restart.
ZonePauseClimate::HoldKind ZonePauseClimate::restore_hold_kind_() const {
  const uint16_t hold = this->data_.hold_minutes;
  if (hold == 0)
    return HOLD_KIND_NONE;
  if (hold >= HOLD_PERMANENT)
    return HOLD_KIND_PERMANENT;
  if (!this->restarted_since_pause_ && hold > this->paused_minutes_ + MIN_TIMED_HOLD)
    return HOLD_KIND_TIMED;
  return HOLD_KIND_NONE;
}

void ZonePauseClimate::set_goal_(Goal goal) {
  this->data_.goal = goal;
  this->goal_seen_[HEAT] = this->goal_seen_[COOL] = false;
  this->resend_count_ = 0;
  if (goal == GOAL_NONE) {
    this->send_due_ = false;
    this->sent_count_[HEAT] = this->sent_count_[COOL] = 0;
    this->data_.owed_heat = this->data_.owed_cool = this->data_.owed_hold = false;
  }
}

void ZonePauseClimate::add_sent_(Side side, uint8_t value) {
  const uint32_t now = millis();
  const uint8_t capacity = sizeof(this->sent_[side]);
  uint8_t slot = 0;
  for (uint8_t i = 0; i < this->sent_count_[side]; i++) {
    if (this->sent_[side][i] == value) {
      this->sent_ms_[side][i] = now;
      return;
    }
    if (now - this->sent_ms_[side][i] > now - this->sent_ms_[side][slot])
      slot = i;  // the oldest entry
  }
  if (this->sent_count_[side] < capacity)
    slot = this->sent_count_[side]++;
  this->sent_[side][slot] = value;
  this->sent_ms_[side][slot] = now;
}

// A value one of our own sends carried recently; it may be showing up late.
bool ZonePauseClimate::is_sent_(Side side, uint8_t value) const {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < this->sent_count_[side]; i++) {
    if (this->sent_[side][i] == value && now - this->sent_ms_[side][i] < SENT_EXCUSE_MS)
      return true;
  }
  return false;
}

// Nothing left for a send to do in the current mode.
bool ZonePauseClimate::satisfied_(const uint8_t real[2], bool permanent) const {
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const bool wanted = this->data_.goal == GOAL_PAUSE || this->owed_(side);
    if (lands[s] && wanted && real[s] != this->goal_value_(side))
      return false;
  }
  if (this->data_.goal == GOAL_PAUSE)
    return permanent;
  return !this->data_.owed_hold;
}

void ZonePauseClimate::control(const climate::ClimateCall &call) {
  this->ensure_started_();
  const uint8_t zone = this->source_->get_zone();
  auto fwd = this->source_->make_call();
  bool forward = false;
  bool changed = false;

  // Mode and fan are not affected by pause. They always go straight through; the hub
  // writes them with their own change flags only.
  if (call.get_mode().has_value()) {
    fwd.set_mode(*call.get_mode());
    forward = true;
  }
  if (call.get_fan_mode().has_value()) {
    fwd.set_fan_mode(*call.get_fan_mode());
    forward = true;
  }

  // This entity declares two-point targets, so ESPHome removes a lone target_temperature
  // from every call before it gets here; only low and high can arrive.
  const optional<float> low = call.get_target_temperature_low();
  const optional<float> high = call.get_target_temperature_high();
  if (low.has_value() || high.has_value()) {
    if (this->data_.goal == GOAL_NONE) {
      if (low.has_value())
        fwd.set_target_temperature_low(*low);
      if (high.has_value())
        fwd.set_target_temperature_high(*high);
      forward = true;
    } else {
      // Never forwarded while paused or while settings are being put back: the source sends
      // both setpoints from its own cache, which holds the real values, a wide one included.
      const uint8_t heat = low.has_value() ? this->parent_->celsius_to_setpoint(*low) : this->data_.target_heat;
      const uint8_t cool = high.has_value() ? this->parent_->celsius_to_setpoint(*high) : this->data_.target_cool;
      if (heat + MIN_GAP > cool) {
        ESP_LOGW(TAG, "Zone %d: target %d / %d rejected, heat and cool must be at least %d apart", zone, heat, cool,
                 MIN_GAP);
      } else {
        this->data_.target_heat = heat;
        this->data_.target_cool = cool;
        this->data_.target_changed = true;
        changed = true;
        if (this->data_.goal == GOAL_PAUSE) {
          ESP_LOGI(TAG, "Zone %d is paused: target now %d / %d, applied when the pause ends", zone, heat, cool);
        } else {
          if (low.has_value())
            this->data_.owed_heat = true;
          if (high.has_value())
            this->data_.owed_cool = true;
          this->send_due_ = true;
          this->resend_count_ = 0;
          ESP_LOGI(TAG, "Zone %d: target now %d / %d (settings were still being put back)", zone, heat, cool);
        }
      }
    }
  }

  const bool has_preset = call.has_custom_preset() || call.get_preset().has_value();
  if (has_preset) {
    if (this->data_.goal == GOAL_PAUSE) {
      ESP_LOGW(TAG, "Zone %d is paused: preset ignored. Turn the pause off first.", zone);
    } else {
      if (this->data_.goal == GOAL_RESTORE) {
        // The person's action wins over the hold being put back. The setpoints stay owed, so
        // the preset's own values landing are seen as changes and end what is owed; nothing
        // wide can be left behind.
        this->data_.owed_hold = false;
        this->data_.restore_hold = false;
        changed = true;
      }
      if (call.has_custom_preset()) {
        auto custom = call.get_custom_preset();
        fwd.set_preset(custom.c_str(), custom.size());
      } else {
        fwd.set_preset(*call.get_preset());
      }
      forward = true;
    }
  }

  if (changed)
    this->save_();
  // The source publishes at the end of every call it handles, and its state callback
  // already republishes this entity, so publish here only when nothing was forwarded.
  if (forward)
    fwd.perform();
  else
    this->publish_all_();
  if (this->send_due_)
    this->evaluate_();
}

void ZonePauseClimate::request_pause(bool pause) {
  this->ensure_started_();
  const uint8_t zone = this->source_->get_zone();

  // Pausing a paused zone keeps the snapshot (it must never remember the wide values);
  // unpausing a running zone does nothing.
  if (pause == this->data_.paused) {
    this->publish_all_();
    return;
  }

  if (pause) {
    uint8_t real[2] = {0, 0};
    bool permanent = false;
    uint16_t hold_minutes = 0;
    if (!this->parent_->is_bus_online()) {
      ESP_LOGW(TAG, "Zone %d: pause refused, the Carrier bus is offline (check the board's bus wiring and power)", zone);
      this->publish_all_();
      return;
    }
    if (!this->bus_ready_() || !this->read_real_(real, permanent, &hold_minutes) || !this->real_is_fresh_()) {
      ESP_LOGW(TAG, "Zone %d: pause refused, no recent answer from the thermostat yet. Try again in a few seconds.",
               zone);
      this->publish_all_();
      return;
    }
    if (this->data_.goal == GOAL_RESTORE) {
      // An earlier unpause has not finished. What is still owed may still be wide, so keep
      // the remembered value for it; everything else is remembered as it really is now.
      if (!this->data_.owed_heat)
        this->data_.target_heat = real[HEAT];
      if (!this->data_.owed_cool)
        this->data_.target_cool = real[COOL];
      if (!this->data_.owed_hold)
        this->data_.hold_minutes = hold_minutes;
      ESP_LOGI(TAG, "Zone %d: PAUSE again. Remembering %d / %d (hold %u)", zone, this->data_.target_heat,
               this->data_.target_cool, this->data_.hold_minutes);
    } else {
      this->data_.target_heat = real[HEAT];
      this->data_.target_cool = real[COOL];
      this->data_.hold_minutes = hold_minutes;
      this->data_.target_changed = false;
      this->restarted_since_pause_ = false;
      this->paused_minutes_ = 0;
      this->minute_accum_ms_ = 0;
      if (!this->in_quiet_) {
        this->baseline_[HEAT] = real[HEAT];
        this->baseline_[COOL] = real[COOL];
      }
      ESP_LOGI(TAG, "Zone %d: PAUSE. Remembering %d / %d (hold %u)", zone, real[HEAT], real[COOL], hold_minutes);
    }
    this->data_.wide_heat = this->pause_heat_bus_();
    this->data_.wide_cool = this->pause_cool_bus_();
    this->data_.paused = true;
    this->set_goal_(GOAL_PAUSE);
    this->data_.owed_heat = this->data_.owed_cool = this->data_.owed_hold = false;
    this->confirmed_mode_ = this->read_confirmed_mode_();
  } else {
    ESP_LOGI(TAG, "Zone %d: UNPAUSE. Putting back %d / %d (hold %u, target edited: %s)", zone, this->data_.target_heat,
             this->data_.target_cool, this->data_.hold_minutes, YESNO(this->data_.target_changed));
    this->data_.paused = false;
    this->set_goal_(GOAL_RESTORE);
    this->data_.owed_heat = this->data_.owed_cool = true;
    this->data_.owed_hold = true;
    this->data_.restore_hold = true;
  }
  this->send_due_ = true;
  this->save_();
  this->publish_all_();
  this->evaluate_();
}

// Builds and issues the send for the goal as it is right now. The hub calls run back to
// back in one call stack: each updates the hub's working copy of the zones register before
// returning, so the second command is built on top of the first, and the hub's first-in
// first-out retry queue puts the one queued second last on the wire every time. Do not
// split them across loop iterations. Re-check that queue whenever the InfinitESP pin moves.
void ZonePauseClimate::issue_send_(const uint8_t real[2]) {
  const uint8_t zone = this->source_->get_zone();
  uint8_t heat, cool;
  if (this->data_.goal == GOAL_PAUSE) {
    heat = this->data_.wide_heat;
    cool = this->data_.wide_cool;
    this->parent_->set_zone_setpoint(zone, heat, cool);
    // Always sent, also on a zone that is already held: whether a setpoint write alone
    // keeps a permanent hold permanent is not verified.
    this->parent_->set_zone_hold(zone, HOLD_PERMANENT);
    ESP_LOGI(TAG, "Zone %d: sent pause %d / %d with a permanent hold", zone, heat, cool);
  } else {
    // A side that is no longer owed goes out as its real value.
    heat = this->data_.owed_heat ? this->data_.target_heat : real[HEAT];
    cool = this->data_.owed_cool ? this->data_.target_cool : real[COOL];
    const HoldKind kind = this->restore_hold_kind_();
    // The hold is touched only while it is still owed; once it reads as wanted it is left alone.
    if (!this->data_.restore_hold || !this->data_.owed_hold) {
      this->parent_->set_zone_setpoint(zone, heat, cool);
    } else if (kind == HOLD_KIND_PERMANENT) {
      this->parent_->set_zone_setpoint(zone, heat, cool);
      this->parent_->set_zone_hold(zone, HOLD_PERMANENT);
    } else if (kind == HOLD_KIND_TIMED) {
      this->parent_->set_zone_setpoint(zone, heat, cool);
      this->parent_->set_zone_hold(zone, this->data_.hold_minutes - this->paused_minutes_);
    } else if (this->data_.target_changed) {
      // Scheduled zone, target edited: leave the pause hold first, then set the values, so
      // the thermostat starts its usual hold at them like any setpoint change.
      this->parent_->set_zone_hold(zone, 0);
      this->parent_->set_zone_setpoint(zone, heat, cool);
    } else {
      // Scheduled zone: values back, then unhold. The thermostat then loads its schedule
      // values, or keeps these; either way the zone is not left wide.
      this->parent_->set_zone_setpoint(zone, heat, cool);
      this->parent_->set_zone_hold(zone, 0);
    }
    ESP_LOGI(TAG, "Zone %d: sent restore %d / %d (still owed: heat %s, cool %s, hold %s)", zone, heat, cool,
             YESNO(this->data_.owed_heat), YESNO(this->data_.owed_cool), YESNO(this->data_.owed_hold));
  }
  this->add_sent_(HEAT, heat);
  this->add_sent_(COOL, cool);
  this->baseline_[HEAT] = real[HEAT];
  this->baseline_[COOL] = real[COOL];
  this->goal_seen_[HEAT] = this->goal_seen_[COOL] = false;
  this->last_send_ms_ = millis();
  this->ever_sent_ = true;
  this->in_quiet_ = true;
  this->send_due_ = false;
}

// A watched setpoint changed to something that is not ours while the zone was paused: the
// pause is over and that value stands. Nobody tries to work out who did it.
void ZonePauseClimate::end_pause_by_deviation_(const uint8_t real[2], const bool deviated[2]) {
  const uint8_t zone = this->source_->get_zone();
  ESP_LOGW(TAG, "Zone %d: PAUSE ENDED BY ITSELF: the thermostat's setpoints were changed to %d / %d by something "
                "else. Those values stand.",
           zone, real[HEAT], real[COOL]);
  this->data_.paused = false;
  bool owed[2] = {false, false};
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const uint8_t wide = side == HEAT ? this->data_.wide_heat : this->data_.wide_cool;
    const uint8_t target = side == HEAT ? this->data_.target_heat : this->data_.target_cool;
    if (deviated[s])
      this->adopt_target_(side, real[s]);  // their value is the target from now on
    else if (RESTORE_UNTOUCHED_SIDES && real[s] == wide && wide != target)
      owed[s] = true;
  }
  if (owed[HEAT] || owed[COOL]) {
    this->set_goal_(GOAL_RESTORE);
    this->data_.owed_heat = owed[HEAT];
    this->data_.owed_cool = owed[COOL];
    // A zone that was on a permanent hold keeps it. For a scheduled zone the pause's hold is
    // left alone: releasing it would discard the value that was just set.
    this->data_.restore_hold = this->restore_hold_kind_() == HOLD_KIND_PERMANENT;
    this->data_.owed_hold = this->data_.restore_hold;
    this->send_due_ = true;
    ESP_LOGI(TAG, "Zone %d: putting back the side nobody touched (heat %s, cool %s)", zone, YESNO(owed[HEAT]),
             YESNO(owed[COOL]));
  } else {
    this->set_goal_(GOAL_NONE);
  }
  this->save_();
  this->publish_all_();
}

// Judged only outside quiet periods: once when one ends and then on every evaluation.
void ZonePauseClimate::judge_(const uint8_t real[2], bool permanent, bool quiet_period_just_ended) {
  const uint8_t zone = this->source_->get_zone();
  bool watched[2];
  watched_sides_(this->confirmed_mode_, watched[HEAT], watched[COOL]);
  bool deviated[2] = {false, false};
  bool changed = false;

  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const uint8_t value = real[s];
    const uint8_t wide = side == HEAT ? this->data_.wide_heat : this->data_.wide_cool;
    if (value == this->goal_value_(side)) {
      // It landed (or never had to move).
      this->baseline_[s] = value;
      if (this->data_.goal == GOAL_RESTORE && this->owed_(side)) {
        this->set_owed_(side, false);
        changed = true;
        ESP_LOGI(TAG, "Zone %d: %s is back at %d", zone, SIDE_NAMES[s], value);
      }
    } else if (value == this->baseline_[s]) {
      // Has not moved: a send that did not land, or a mode that ignores this side.
    } else if (this->is_sent_(side, value) || (this->data_.goal == GOAL_RESTORE && value == wide)) {
      // A send of ours showing up late. The next send corrects it.
      this->baseline_[s] = value;
    } else if (this->data_.goal == GOAL_PAUSE && !watched[s]) {
      // The current mode does not use this side: keep the new value as its target.
      this->baseline_[s] = value;
      this->adopt_target_(side, value);
      changed = true;
      ESP_LOGI(TAG, "Zone %d is paused: %s was changed to %d by something else; kept as the new %s target", zone,
               SIDE_NAMES[s], value, SIDE_NAMES[s]);
    } else {
      deviated[s] = true;
      this->baseline_[s] = value;
    }
  }

  if (this->data_.goal == GOAL_PAUSE) {
    if (deviated[HEAT] || deviated[COOL]) {
      this->end_pause_by_deviation_(real, deviated);
      return;
    }
  } else {
    for (uint8_t s = 0; s < 2; s++) {
      const Side side = static_cast<Side>(s);
      if (!deviated[s])
        continue;
      if (this->owed_(side)) {
        this->set_owed_(side, false);
        ESP_LOGI(TAG, "Zone %d: %s was changed to %d by something else while being put back; that value stands", zone,
                 SIDE_NAMES[s], real[s]);
      }
      this->adopt_target_(side, real[s]);
      changed = true;
    }
    if (this->data_.owed_hold) {
      const bool want_permanent = this->restore_hold_kind_() == HOLD_KIND_PERMANENT;
      if (permanent == want_permanent) {
        this->data_.owed_hold = false;
        changed = true;
      }
    }
    if (!this->data_.owed_heat && !this->data_.owed_cool && !this->data_.owed_hold) {
      ESP_LOGI(TAG, "Zone %d: settings are back (%d / %d)", zone, real[HEAT], real[COOL]);
      this->set_goal_(GOAL_NONE);
      changed = true;
    }
  }

  // A wanted value that did not land in a mode that takes it: send again a few times.
  // Nothing ends because of this; the next mode change sends again anyway.
  if (quiet_period_just_ended && this->data_.goal != GOAL_NONE && !this->satisfied_(real, permanent)) {
    bool lands[2];
    landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
    if (lands[HEAT] || lands[COOL]) {
      if (this->resend_count_ < MAX_RESENDS) {
        this->resend_count_++;
        this->send_due_ = true;
        ESP_LOGW(TAG, "Zone %d: the thermostat has not taken %d / %d yet (it shows %d / %d); sending again (%d of %d)",
                 zone, this->goal_value_(HEAT), this->goal_value_(COOL), real[HEAT], real[COOL], this->resend_count_,
                 MAX_RESENDS);
      } else if (this->resend_count_ == MAX_RESENDS) {
        this->resend_count_++;  // log once
        ESP_LOGE(TAG, "Zone %d: the thermostat still shows %d / %d, not %d / %d. Giving up until the system mode "
                      "changes. Check the Actual setpoint sensors.",
                 zone, real[HEAT], real[COOL], this->goal_value_(HEAT), this->goal_value_(COOL));
      }
    }
  }

  if (changed) {
    this->save_();
    this->publish_all_();
  }
}

// After a restart there is no baseline and nothing of ours is in flight (the hub's retry
// queue is gone too). A value that is neither the wide one nor the remembered one was
// changed while the board was down.
void ZonePauseClimate::reconcile_after_restart_(const uint8_t real[2]) {
  const uint8_t zone = this->source_->get_zone();
  const uint8_t wide[2] = {this->data_.wide_heat, this->data_.wide_cool};
  const uint8_t target[2] = {this->data_.target_heat, this->data_.target_cool};
  bool watched[2];
  watched_sides_(this->confirmed_mode_, watched[HEAT], watched[COOL]);
  bool deviated[2] = {false, false};
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    this->baseline_[s] = real[s];
    if (real[s] == wide[s] || real[s] == target[s])
      continue;
    if (this->data_.goal == GOAL_PAUSE && watched[s]) {
      deviated[s] = true;
    } else {
      this->set_owed_(side, false);
      this->adopt_target_(side, real[s]);
    }
  }
  if (this->data_.goal == GOAL_PAUSE && (deviated[HEAT] || deviated[COOL])) {
    this->end_pause_by_deviation_(real, deviated);
    return;
  }
  ESP_LOGI(TAG, "Zone %d: %s after the restart", zone,
           this->data_.goal == GOAL_PAUSE ? "still paused" : "still putting settings back");
  this->send_due_ = true;
  this->save_();
  this->publish_all_();
}

void ZonePauseClimate::evaluate_() {
  uint8_t real[2] = {0, 0};
  bool permanent = false;
  const bool have = this->read_real_(real, permanent);
  if (have)
    this->publish_actual_(real[HEAT], real[COOL]);
  const uint8_t mode = this->read_confirmed_mode_();
  const uint32_t now = millis();

  if (this->needs_reconcile_) {
    if (!this->bus_ready_() || !have || !this->real_is_fresh_() || mode == MODE_UNKNOWN)
      return;
    this->needs_reconcile_ = false;
    this->confirmed_mode_ = mode;
    this->reconcile_after_restart_(real);
  }

  if (this->data_.goal == GOAL_NONE) {
    this->confirmed_mode_ = mode;
    return;
  }
  if (!have)
    return;

  // Judge first, under the mode the values were seen in, so that a change by something
  // else is never folded into a fresh baseline by a send that happens to be due.
  if (this->in_quiet_) {
    if (now - this->last_send_ms_ < QUIET_MS) {
      for (uint8_t s = 0; s < 2; s++) {
        if (real[s] == this->goal_value_(static_cast<Side>(s)))
          this->goal_seen_[s] = true;
      }
    } else {
      this->in_quiet_ = false;
      // A side that reached the goal value counts from there, so a change made after our
      // write landed is still a change.
      for (uint8_t s = 0; s < 2; s++) {
        if (this->goal_seen_[s])
          this->baseline_[s] = this->goal_value_(static_cast<Side>(s));
      }
      this->judge_(real, permanent, /*quiet_period_just_ended=*/true);
    }
  } else {
    this->judge_(real, permanent, /*quiet_period_just_ended=*/false);
  }
  if (this->data_.goal == GOAL_NONE)
    return;

  if (mode != MODE_UNKNOWN && mode != this->confirmed_mode_) {
    ESP_LOGI(TAG, "Zone %d: system mode changed from %s to %s", this->source_->get_zone(),
             mode_name_(this->confirmed_mode_), mode_name_(mode));
    this->confirmed_mode_ = mode;
    this->send_due_ = true;
    this->resend_count_ = 0;
  }
  if (!this->send_due_)
    return;
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  // Nothing is sent into a mode where nothing lands; the send stays due.
  if (!lands[HEAT] && !lands[COOL])
    return;
  if (this->satisfied_(real, permanent) && !this->in_quiet_) {
    this->send_due_ = false;  // already where it should be; nothing to send
    return;
  }
  const bool gap_over = !this->ever_sent_ || now - this->last_send_ms_ >= SEND_GAP_MS;
  if (gap_over && this->bus_ready_() && this->real_is_fresh_())
    this->issue_send_(real);
}

void ZonePauseClimate::mirror_from_source_() {
  this->current_temperature = this->source_->current_temperature;
  this->mode = this->source_->mode;
  this->action = this->source_->action;
  this->fan_mode = this->source_->fan_mode;

  if (this->data_.goal != GOAL_NONE) {
    // Show the target, never the wide values: all of it while paused, and afterwards for
    // each side that has not been put back yet.
    const bool show_heat = this->data_.goal == GOAL_PAUSE || this->data_.owed_heat;
    const bool show_cool = this->data_.goal == GOAL_PAUSE || this->data_.owed_cool;
    this->target_temperature_low = show_heat ? this->parent_->setpoint_to_celsius(this->data_.target_heat)
                                             : this->source_->target_temperature_low;
    this->target_temperature_high = show_cool ? this->parent_->setpoint_to_celsius(this->data_.target_cool)
                                              : this->source_->target_temperature_high;
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

// save() alone only queues the data; ESPHome writes its queue to flash once a minute. A
// power cut inside that minute would forget a pause while the thermostat keeps the zone
// held wide, so it is written through straight away, except that writes closer together
// than SYNC_GAP_MS are merged into one.
void ZonePauseClimate::save_() {
  this->dirty_ = true;
  this->flush_();
}

void ZonePauseClimate::flush_() {
  const uint32_t now = millis();
  if (this->last_sync_ms_ != 0 && now - this->last_sync_ms_ < SYNC_GAP_MS)
    return;  // the 1 Hz tick picks it up
  this->dirty_ = false;
  this->last_sync_ms_ = now == 0 ? 1 : now;
  if (this->pref_.save(&this->data_))
    global_preferences->sync();
}

}  // namespace zone_pause
}  // namespace esphome
