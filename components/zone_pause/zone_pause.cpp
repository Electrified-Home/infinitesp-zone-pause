#include "zone_pause.h"
#include "esphome/core/controller_registry.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstdio>
#include <string>

namespace esphome {
namespace zone_pause {

static const char *const TAG = "zone_pause";
static const uint8_t SAVED_VERSION = 6;
static const uint32_t SAVED_KEY = 0x5A4F4E46UL;
static const uint32_t TICK_INTERVAL_MS = 1000;
// One bus write at a time, house-wide: every setpoint, fan or hold write rewrites the whole
// zones register, and the thermostat drops a write that another one follows within a
// fraction of a second. The hub repeats every write 3 times, 4 s apart, from a payload
// captured when it was queued; 10 s clears that span. Shared by all zones.
static const uint32_t SEND_GAP_MS = 10000;
// Nothing is judged for this long after a write. The write is still being repeated (last
// attempt up to about 12 s after it was queued) and the thermostat's own reply was seen to
// lag a change by up to 15 s, so until then a real value can still be on its way to, or
// back from, a value of ours. Nothing visible waits for this.
static const uint32_t QUIET_MS = 30000;
// A value the current send chain sent can show up late. It stays ours until this long after the
// zone's LAST write (the first judgement comes QUIET_MS after that write), so a later stage held
// up by the shared gate never makes an earlier one look like somebody else's. Only the current
// chain's values count (issue_send_): an older chain's are already older than this when judged,
// so a person can set one again.
static const uint32_t SENT_EXCUSE_MS = 60000;
// A hold (or cancel) this component wrote defines the hold state for this long; after that
// the thermostat's own reply does.
static const uint32_t OWN_HOLD_MS = 30000;
// Sends repeated because a wanted value did not land, per goal and per mode change.
static const uint8_t MAX_RESENDS = 1;
// Real values older than this are not trusted for a snapshot or a send (polls come about
// every 6 s).
static const uint32_t FRESH_MS = 20000;
// Flash writes closer together than this are merged (a flapping switch).
static const uint32_t SYNC_GAP_MS = 2000;
// The thermostat ignores timed holds shorter than this (InfinitESP HOLD_TIMED_MIN); the hub
// rounds to its 15-minute grid and clamps to this range, so nothing here clamps again. It is
// the slack used when judging whether a timed hold landed.
static const uint16_t MIN_TIMED_HOLD = 15;
// The longest countdown the thermostat runs (23:59). What the board writes is on the hub's
// grid, at most HOLD_TIMED_MAX (1425).
static const uint16_t MAX_COUNTDOWN = 1439;
static const uint16_t MAX_WRITTEN_HOLD = infinitesp::InfinitESPComponent::HOLD_TIMED_MAX;
// A change always lasts at least this long (owner policy); a restore is not a change.
static const uint16_t MIN_CHANGE_HOLD = 30;
// The thermostat's installer deadband (bus units): it keeps cool at least this far above heat.
// Carrier's default is 2 (adjustable 0-6); this thermostat reads 2 (cfgdead).
// If the installer setting changes, change this too: nothing here would notice.
static const uint8_t SETPOINT_GAP = 2;
// Schedule row: read at boot (staggered per zone) and then this often; stale after twice that.
static const uint32_t SCHEDULE_REFRESH_MS = 6UL * 3600UL * 1000UL;
static const uint32_t SCHEDULE_STALE_MS = 12UL * 3600UL * 1000UL;

static const uint8_t MODE_UNKNOWN = 0xFF;
static const uint16_t HOLD_PERMANENT = infinitesp::InfinitESPComponent::HOLD_PERMANENT;
// A timed target is kept as the minute of the week it ends on (Sunday 00:00 = 0).
static const uint16_t MINUTES_PER_WEEK = 7 * 1440;
static const uint16_t HOLD_END_UNKNOWN = 0xFFFF;
// minutes_left_() when the thermostat's clock cannot be read.
static const uint16_t LEFT_CLOCK_UNKNOWN = 0xFFFF;

// The target's end and the time left to it, on the week's minutes.
static constexpr uint16_t end_of_(uint16_t now_mow, uint16_t minutes) {
  return (uint16_t) (((uint32_t) now_mow + minutes) % MINUTES_PER_WEEK);
}
static constexpr uint16_t left_(uint16_t now_mow, uint16_t end) {
  return (uint16_t) (((uint32_t) end + MINUTES_PER_WEEK - now_mow) % MINUTES_PER_WEEK);
}
// A timed end still ahead: 1 to MAX_COUNTDOWN minutes left. A passed end wraps above that.
static constexpr bool valid_left_(uint16_t left) { return left >= 1 && left <= MAX_COUNTDOWN; }
// The week wrap, checked when this file compiles.
static_assert(end_of_(6 * 1440 + 1430, 30) == 20, "Saturday 23:50 + 30 ends Sunday 00:20");
static_assert(left_(6 * 1440 + 1430, 20) == 30 && valid_left_(30), "and is 30 minutes away");
static_assert(left_(6 * 1440 + 1430, 6 * 1440 + 1425) == 10075 && !valid_left_(10075), "an end 5 minutes past");
static_assert(left_(6 * 1440 + 1430, 6 * 1440 + 1430) == 0 && !valid_left_(0), "an end equal to now");
// Hold Until: the minutes from now to the next time the clock reads `target`, both minutes of the day.
// 1 to 1440, never 0 (that would be the schedule): a time already passed, or this very minute, is tomorrow's.
static constexpr uint16_t until_(uint16_t target, uint16_t now_minute) {
  return (uint16_t) ((target + 1439 - now_minute) % 1440 + 1);
}
static_assert(until_(17 * 60 + 30, 15 * 60 + 7) == 143, "17:30 from 15:07 is 2 h 23 min");
static_assert(until_(15 * 60 + 8, 15 * 60 + 7) == 1 && until_(15 * 60 + 6, 15 * 60 + 7) == 1439,
              "the next minute is 1; the minute before is tomorrow's");
static_assert(until_(15 * 60 + 7, 15 * 60 + 7) == 1440, "this very minute is tomorrow's, not 0");
static_assert(until_(0, 23 * 60 + 59) == 1 && until_(23 * 60 + 59, 0) == 1439, "across midnight");
// Two minute counts the hub's nearest-15 rounding puts on the same quarter hour.
static bool same_quarter_(uint16_t a, uint16_t b) {
  return infinitesp::InfinitESPComponent::normalize_timed_hold(a) ==
         infinitesp::InfinitESPComponent::normalize_timed_hold(b);
}
// A target's hold for the log: "no hold", "hold until cancelled" or "hold 60 min, ends 14:30".
static const char *hold_text_(uint16_t minutes, uint16_t end) {
  static char buf[40];  // shared: one call per log line
  if (minutes == 0 || minutes >= HOLD_PERMANENT)
    return minutes == 0 ? "no hold" : "hold until cancelled";
  const int n = snprintf(buf, sizeof(buf), "hold %u min", (unsigned) minutes);
  if (end != HOLD_END_UNKNOWN)
    snprintf(buf + n, sizeof(buf) - n, ", ends %02u:%02u", (unsigned) (end % 1440 / 60), (unsigned) (end % 60));
  return buf;
}
// The card's own preset for a paused zone. The pause switch is still the automation handle.
static const char *const PRESET_PAUSED = "Paused";
static const char *const SIDE_NAMES[2] = {"heat", "cool"};
static const char *const ACTIVITY_NAMES[5] = {"home", "away", "sleep", "wake", "manual"};

// The write gate is shared by every zone (one whole-register write on the bus per gap).
static uint32_t s_last_write_ms = 0;
static bool s_ever_wrote = false;
static bool gate_open_() { return !s_ever_wrote || millis() - s_last_write_ms >= SEND_GAP_MS; }

void ZonePauseSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->request_pause(state);
}


void ZonePauseHoldMinutes::control(float value) {
  if (this->parent_ != nullptr)
    this->parent_->hold_minutes_write(value);
}

#ifdef USE_DATETIME_TIME
// The seconds of a write are ignored.
void ZonePauseHoldUntil::control(const datetime::TimeCall &call) {
  const auto hour = call.get_hour();
  const auto minute = call.get_minute();
  if (this->parent_ != nullptr && hour.has_value() && minute.has_value())
    this->parent_->hold_until_write(*hour, *minute);
}

// Published only when it changes. With no timed hold a time entity has no 0 to show (00:00 would
// read as "until midnight"), so it goes unknown, which Home Assistant shows as such. publish_state()
// cannot do that: it tells nobody when the time is out of range, so the state is cleared here and
// the frontends are told with the same call publish_state() ends with.
void ZonePauseHoldUntil::show(int minute_of_day) {
  if (minute_of_day < 0) {
    if (this->has_state()) {
      this->set_has_state(false);
#ifdef USE_CONTROLLER_REGISTRY
      ControllerRegistry::notify_time_update(this);
#endif
    }
    return;
  }
  // An idle zone's end is worked out from the thermostat's clock and its hold countdown, which the hub
  // reads about 3 s apart, so around each minute change it wobbles by a minute. A move of one minute
  // (or none) is not shown; holds change by more than that.
  if (this->has_state()) {
    const int apart = (minute_of_day - (this->hour_ * 60 + this->minute_) + 1440) % 1440;
    if (apart <= 1 || apart >= 1439)
      return;
  }
  const uint8_t hour = minute_of_day / 60;
  const uint8_t minute = minute_of_day % 60;
  this->hour_ = hour;
  this->minute_ = minute;
  this->second_ = 0;
  this->publish_state();
}
#endif

// Hold Minutes is how long the target holds, in any state: it goes through the shared gate
// like every write and applies even when the temperature is already there. With a temperature
// (N > 0), in either order, the two form one target; 0 means the schedule (a temperature given
// before it is dropped, as Per Schedule does). One that changes nothing is dismissed.
void ZonePauseClimate::hold_minutes_write(float minutes) {
  this->ensure_started_();
  const uint8_t zone = this->source_->get_zone();
  if (std::isnan(minutes))
    return;
  const int asked = (int) lroundf(fminf(fmaxf(minutes, 0.0f), (float) MAX_WRITTEN_HOLD));
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  // Paused or in OFF nothing goes out now: the hold asked for gets no 30-minute minimum.
  const bool goes_out_now = !this->data_.paused && (lands[HEAT] || lands[COOL]);
  Held h{};
  if (!this->hold_for_change_(asked, h, goes_out_now)) {
    this->publish_all_();
    return;
  }
  if (!h.owed) {
    ESP_LOGD(TAG, "Zone %d: Hold Minutes %d: the target already holds that; nothing to do", zone, asked);
    this->publish_all_();
    return;
  }
  this->drop_why_ = nullptr;
  if (this->data_.paused) {
    this->store_hold_(h);  // the unpause applies it, or the schedule if it ran out by then
    ESP_LOGI(TAG, "Zone %d is paused: the target now has %s, applied when the pause ends", zone,
             hold_text_(h.minutes, h.end));
  } else if (asked == 0) {
    this->to_schedule_();
    ESP_LOGI(TAG, "Zone %d: Hold Minutes 0: back to the schedule", zone);
  } else {
    // Every side the mode takes is owed at its target, even one already there (the timed-hold
    // write reloads the schedule's values). In OFF the target's values wait, and its hold with
    // them (deliver_desired_ keeps them until a mode takes a side).
    this->begin_set_goal_();
    for (uint8_t s = 0; s < 2; s++) {
      const Side side = static_cast<Side>(s);
      if (lands[s])
        this->set_owed_(side, true);
      else if (!lands[HEAT] && !lands[COOL] && this->desired_(side) == 0)
        this->set_desired_(side, this->goal_value_(side));
    }
    this->store_hold_(h);
    if (this->data_.owed_heat || this->data_.owed_cool)
      this->send_due_ = true;
    ESP_LOGI(TAG, "Zone %d: Hold Minutes %d: target %d / %d, %s", zone, asked, this->data_.target_heat,
             this->data_.target_cool, hold_text_(h.minutes, h.end));
  }
  this->save_();
  this->publish_all_();
  if (this->send_due_)
    this->evaluate_();
}

// Hold Until is Hold Minutes set by the clock: the minutes from now to the NEXT time the
// thermostat's clock reads hour:minute (1-1440: a time already passed, or this very minute, is
// tomorrow's), then exactly what setting Hold Minutes to that number does. So it has the same
// grid, rounding, limits (1440 is held as 1425), 30-minute minimum when the change goes out, and
// paused rule. The grid counts the minutes from now, so the end is at the time asked or up to 14 minutes after it,
// never before (the 1425 maximum and the 30-minute minimum aside); Hold Until then shows the end it really has.
void ZonePauseClimate::hold_until_write(uint8_t hour, uint8_t minute) {
  this->ensure_started_();
  uint16_t now = 0;
  if (!this->now_mow_(now)) {
    ESP_LOGE(TAG, "Zone %d: change refused: the thermostat's clock is not known yet, so Hold Until cannot be timed",
             this->source_->get_zone());
    this->publish_all_();
    return;
  }
  // Up to the hold's 15-minute grid (owner, 2026-09-30: "7:05" ends at the first grid point at or after it), so the hold
  // never ends before the time asked; Hold Minutes' own nearest-15 rounding then leaves it as it is.
  const uint16_t want = until_(hour * 60 + minute, now % 1440);
  uint16_t asked = (want + 14) / 15 * 15;
  // A running timed hold in the same quarter keeps its own end (hold_for_change_), which can be up to 14 minutes before
  // the time asked: step to the next quarter so it is written and never ends early (the 1425 maximum aside).
  const uint16_t met = this->data_.hold_minutes;
  if (met != 0 && met < HOLD_PERMANENT) {
    const uint16_t left = this->minutes_left_(this->data_.hold_end_mow);
    if (valid_left_(left) && left < want && same_quarter_(left, asked) && asked + 15 <= MAX_WRITTEN_HOLD)
      asked += 15;
  }
  ESP_LOGD(TAG, "Zone %d: Hold Until %02u:%02u is Hold Minutes %u (up to the 15-minute grid)", this->source_->get_zone(),
           (unsigned) hour, (unsigned) minute, (unsigned) asked);
  this->hold_minutes_write((float) asked);
}

void ZonePauseClimate::init() {
  this->set_supported_custom_presets({
      infinitesp::PRESET_SCHEDULE,  infinitesp::PRESET_WAKE,     infinitesp::PRESET_HOLD_TIMED,
      infinitesp::PRESET_HOLD_PERM, infinitesp::PRESET_VACATION, PRESET_PAUSED,
  });
  this->data_ = Saved{};
  this->data_.version = SAVED_VERSION;
  this->data_.set_fan = NO_FAN;
  this->data_.set_preset = NO_PRESET;
  this->data_.hold_end_mow = HOLD_END_UNKNOWN;
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
      ESP_LOGE(TAG, "Zone %d: saved pause state is from another version and was ignored. If this zone was paused, "
                    "it may still hold the wide setpoints (%d / %d F) with its Pause switch off: set its "
                    "temperatures by hand.",
               this->source_->get_zone(), this->pause_heat_f_, this->pause_cool_f_);
    } else if (loaded.paused || loaded.goal != GOAL_NONE || loaded.desired_heat != 0 || loaded.desired_cool != 0) {
      this->data_ = loaded;
      this->needs_reconcile_ = true;
      ESP_LOGI(TAG, "Zone %d: restarted while %s; checking against the thermostat", this->source_->get_zone(),
               loaded.paused ? "paused" : (loaded.goal != GOAL_NONE ? "a change was still being applied" : "a value was waiting for a mode change"));
    }
  }
  this->schedule_poll_at_ms_ = millis() + 20000UL * this->source_->get_zone();
  ESP_LOGI(TAG, "Zone %d: wide setpoints %d / %d F, minimum hold for edits %u min, paused: %s",
           this->source_->get_zone(),
           this->pause_heat_f_, this->pause_cool_f_, this->minimum_hold_, YESNO(this->data_.paused));
}

void ZonePauseClimate::on_register_update(uint8_t device_addr, uint16_t register_key) {
  // Mostly a heartbeat (the hub notifies on every register it stores), and the one place
  // that sees when a real reply from the thermostat arrives.
  const uint32_t now = millis();
  if (device_addr == infinitesp::ADDR_THERMOSTAT && register_key == infinitesp::REG_SAM_ZONES) {
    this->have_zones_reply_ = true;
    this->last_zones_reply_ms_ = now;
  }
  if (device_addr == infinitesp::ADDR_THERMOSTAT &&
      register_key == (uint16_t) (infinitesp::REG_TSTAT_SCHEDULE + this->source_->get_zone() - 1)) {
    this->schedule_read_ms_ = now;
    this->schedule_read_valid_ = true;
  }
  const bool first = !this->started_;
  this->ensure_started_();
  if (first)
    this->publish_all_();
  const uint32_t elapsed = now - this->last_tick_ms_;
  if (elapsed < TICK_INTERVAL_MS)
    return;
  this->last_tick_ms_ = now;
  this->minute_accum_ms_ += elapsed;
  bool minute = false;
  while (this->minute_accum_ms_ >= 60000) {
    this->minute_accum_ms_ -= 60000;
    minute = true;
  }
  // The schedule row: one read per zone at boot and every few hours, from the tick,
  // staggered per zone and kept clear of every zone's writes: the bus must have been quiet
  // for a whole send gap plus a quiet period, house-wide (each read costs the hub one
  // missed poll of its own).
  const bool bus_quiet = !s_ever_wrote || (now - s_last_write_ms >= SEND_GAP_MS + QUIET_MS);
  if (this->started_ && (int32_t) (now - this->schedule_poll_at_ms_) >= 0 && this->bus_ready_() && bus_quiet &&
      !this->send_due_ && this->pending_count_ == 0 && !this->pending_mode_valid_) {
    this->schedule_poll_at_ms_ = now + SCHEDULE_REFRESH_MS + 20000UL * this->source_->get_zone();
    const uint8_t row = 0x02 + this->source_->get_zone() - 1;
    this->parent_->poll_register(0x40, row);
    ESP_LOGD(TAG, "Zone %d: asked the thermostat for its schedule row 0x40%02X", this->source_->get_zone(), row);
  }
  if (this->dirty_)
    this->flush_();
  this->evaluate_();
  // The minute tick runs after evaluate_, so a person's change in the second a waiting end passes
  // is caught there first and not taken into the schedule target's baseline. Hold Minutes counts
  // down on its own: once a minute, after evaluate_ has re-seeded an idle target's end, so it
  // never flaps at the minute boundary.
  if (minute) {
    this->minute_tick_();
    this->publish_hold_minutes_();
    this->publish_hold_until_();
  }
}

climate::ClimateTraits ZonePauseClimate::traits() {
  // Same capabilities as the InfinitESP zone it fronts. Its custom presets are registered
  // on the entity in init(), "Paused" among them: the card can pause a zone and shows a
  // paused one as Paused. The pause switch is unchanged and stays the automation handle.
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
bool ZonePauseClimate::read_real_(uint8_t real[2], bool &permanent, uint16_t *hold_minutes, bool *timed,
                                  uint8_t *fan) const {
  const auto *zones = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT, infinitesp::REG_SAM_ZONES);
  if (zones == nullptr || zones->size() < infinitesp::REG3B03_SIZE)
    return false;
  const uint8_t idx = this->source_->get_zone() - 1;
  real[HEAT] = (*zones)[infinitesp::REG3B03_HEAT_SETPOINTS + idx];
  real[COOL] = (*zones)[infinitesp::REG3B03_COOL_SETPOINTS + idx];
  const bool holding = ((*zones)[infinitesp::REG3B03_ZONES_HOLDING] & (1 << idx)) != 0;
  const bool timed_bit = ((*zones)[infinitesp::REG3B03_TIMED_HOLDS] & (1 << idx)) != 0;
  const uint16_t duration = ((uint16_t) (*zones)[infinitesp::REG3B03_HOLD_DURATIONS + idx * 2] << 8) |
                            (*zones)[infinitesp::REG3B03_HOLD_DURATIONS + idx * 2 + 1];
  permanent = holding && duration <= 1;
  if (hold_minutes != nullptr)
    *hold_minutes = permanent ? HOLD_PERMANENT : ((timed_bit && duration == 0) ? 1 : duration);
  if (timed != nullptr)
    *timed = !permanent && (timed_bit || duration > 0);
  if (fan != nullptr)
    *fan = (*zones)[infinitesp::REG3B03_FAN_MODES + idx];
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
  const uint8_t stagmode = (*state)[infinitesp::REG3B02_STAGMODE];
  const uint8_t mode = stagmode & 0x0F;
  // While a stage runs, a Touch control shows an AUTO system's direction (heat, cool, or on a
  // dual-fuel system emergency heat or heat pump) in place of its mode. As InfinitESP does (its
  // climate's mode read), that is ignored while the source climate still shows auto (heat_cool),
  // which follows a mode set from Home Assistant at once. A change at the wall mid-stage is read
  // when the stage ends, as InfinitESP reads it. An off nibble is taken at once.
  if ((stagmode >> 4) != 0 && this->source_->mode == climate::CLIMATE_MODE_HEAT_COOL &&
      (mode == infinitesp::SYSMODE_HEAT || mode == infinitesp::SYSMODE_COOL || mode == infinitesp::SYSMODE_EHEAT ||
       mode == infinitesp::SYSMODE_HEATPUMP))
    return infinitesp::SYSMODE_AUTO;
  return mode;
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

void ZonePauseClimate::set_desired_(Side side, uint8_t value) {
  if (side == HEAT)
    this->data_.desired_heat = value;
  else
    this->data_.desired_cool = value;
}

// The kind of hold a pause restore puts back. A timed hold is remembered by the
// end time it had, so a restart makes no difference; whether that end has passed is decided
// when the hold write is built (issue_send_).
ZonePauseClimate::HoldKind ZonePauseClimate::restore_hold_kind_() const {
  const uint16_t hold = this->data_.hold_minutes;
  if (hold == 0)
    return HOLD_KIND_NONE;
  if (hold >= HOLD_PERMANENT)
    return HOLD_KIND_PERMANENT;
  return this->data_.hold_end_mow == HOLD_END_UNKNOWN ? HOLD_KIND_NONE : HOLD_KIND_TIMED;
}

// The hold state as this component sees it: a hold or cancel it wrote itself in the last
// OWN_HOLD_MS wins (the thermostat's copy lags up to 15 s); otherwise the real reply.
ZonePauseClimate::HoldKind ZonePauseClimate::hold_view_(bool permanent, bool timed, uint16_t hold_minutes) const {
  if (this->own_hold_valid_ && millis() - this->own_hold_ms_ < OWN_HOLD_MS)
    return this->own_hold_kind_;
  if (permanent)
    return HOLD_KIND_PERMANENT;
  if (timed || hold_minutes > 0)
    return HOLD_KIND_TIMED;
  return HOLD_KIND_NONE;
}

void ZonePauseClimate::set_goal_(Goal goal) {
  this->data_.goal = goal;
  this->pending_count_ = 0;  // a new goal supersedes the later stages of the old send
  this->goal_seen_[HEAT] = this->goal_seen_[COOL] = false;
  this->resend_count_ = 0;
  this->hold_fallback_logged_ = false;
  this->hold_sent_ = false;
  // A pause takes its own hold, so a release owed by something else must not leak past it.
  if (goal == GOAL_PAUSE)
    this->data_.release_due = false;
  if (goal == GOAL_NONE) {
    this->send_due_ = false;
    this->reload_expected_ = false;
    this->sent_count_[HEAT] = this->sent_count_[COOL] = 0;
    this->data_.owed_heat = this->data_.owed_cool = this->data_.owed_hold = this->data_.owed_fan = false;
    this->data_.restore_hold = false;
    this->data_.set_fan = NO_FAN;
    this->data_.set_hold = SET_HOLD_KEEP;
    this->data_.set_hold_minutes = 0;
    this->data_.set_preset = NO_PRESET;
    // A due release stays while the target waits (minute_tick_): the zone still shows the hold of
    // the pause whose restore this target took over.
    this->data_.release_due = this->data_.release_due && this->waiting_();
  }
}

// A card action (edit, preset, hold command) starts or joins a SET goal. Coming from NONE
// the targets start as the real values, so an untouched side is sent as it is. Coming from a
// RESTORE, what the restore still owed is kept, its hold included. Every SET carries the
// target's hold; a change with a hold of its own replaces it afterwards (store_hold_).
void ZonePauseClimate::begin_set_goal_() {
  if (this->data_.goal == GOAL_SET) {
    this->pending_count_ = 0;  // latest wins: later stages of the previous send are dropped
    if (this->data_.set_preset != NO_PRESET)
      ESP_LOGI(TAG, "Zone %d: a new card action replaces the %s preset that was still being applied",
               this->source_->get_zone(), ACTIVITY_NAMES[this->data_.set_preset]);
    this->data_.set_preset = NO_PRESET;
    // A Per Schedule whose cancel had not gone out yet is still owed: the release stays due,
    // so the new action writes its own hold instead of keeping a hold nobody wants.
    if (this->data_.set_hold == SET_HOLD_CANCEL && this->data_.owed_hold)
      this->data_.release_due = true;
    // The new action brings its own fan. The owed setpoint sides and the owed hold stay (a
    // bump after Home keeps Home's other side, as the wall would; a temperature that joins a
    // Hold Minutes keeps its hold).
    this->data_.owed_fan = false;
    this->data_.set_fan = NO_FAN;
    this->resend_count_ = 0;
    this->hold_fallback_logged_ = false;
  } else {
    uint8_t real[2] = {0, 0};
    bool permanent = false;
    this->read_real_(real, permanent);
    if (this->data_.goal == GOAL_NONE) {
      this->data_.target_heat = real[HEAT];
      this->data_.target_cool = real[COOL];
      this->data_.owed_heat = this->data_.owed_cool = false;
      // Judging starts from where the thermostat is now, not from a baseline left by an old
      // goal (which could make this very edit look like somebody else's change).
      if (!this->in_quiet_) {
        this->baseline_[HEAT] = real[HEAT];
        this->baseline_[COOL] = real[COOL];
      }
    }
    // From RESTORE: keep the owed sides, their targets, the owed hold and whether its hold write
    // has gone out (then it is judged, not written twice). Until that write goes out, unless
    // the restore was putting a permanent hold back, the hold the zone shows (a pause holds it
    // permanently) is not the hold anybody wants, so a release stays due and the next hold
    // write of this goal carries it out.
    const bool hold_out = this->data_.goal == GOAL_RESTORE && this->hold_sent_;
    if (this->data_.goal == GOAL_RESTORE && this->data_.restore_hold && this->data_.owed_hold && !hold_out &&
        this->restore_hold_kind_() != HOLD_KIND_PERMANENT)
      this->data_.release_due = true;
    this->set_goal_(GOAL_SET);
    this->hold_sent_ = hold_out;
    if (!hold_out)
      this->data_.set_hold_minutes = 0;
    this->data_.restore_hold = false;
    this->data_.owed_fan = false;
    this->data_.set_preset = NO_PRESET;
  }
  this->data_.set_hold = set_hold_of_(this->data_.hold_minutes);
}

uint8_t ZonePauseClimate::set_hold_of_(uint16_t hold_minutes) {
  if (hold_minutes == 0)
    return SET_HOLD_CANCEL;
  return hold_minutes >= HOLD_PERMANENT ? SET_HOLD_PERMANENT : SET_HOLD_TIMED_END;
}

// ---- the target: its hold, a valid pair, and how it ends ------------------------------------

bool ZonePauseClimate::now_mow_(uint16_t &now) const {
  uint8_t weekday = 0;
  uint16_t minutes = 0;
  if (!this->bus_clock_(weekday, minutes))
    return false;
  now = weekday * 1440 + minutes;
  return true;
}

// Minutes from now to a target's end: LEFT_CLOCK_UNKNOWN without the thermostat's clock; an
// unknown end counts as passed (0).
uint16_t ZonePauseClimate::minutes_left_(uint16_t end) const {
  uint16_t now = 0;
  if (end == HOLD_END_UNKNOWN)
    return 0;
  if (!this->now_mow_(now))
    return LEFT_CLOCK_UNKNOWN;
  return left_(now, end);
}

// The hold a change gets. Hold Minutes (`asked` 1-1425) brings its own, always on the hub's
// 15-minute grid (0: the schedule). An edit or a preset meets the target's hold (a passed or
// unknown end counts as the schedule): on the schedule it holds to the next schedule change, and a
// timed hold keeps its end. A change that goes out now lasts at least 30 minutes (an edit on the
// schedule also minimum_hold_), and a hold it writes is on the grid. One that waits (paused, or
// nothing it gives is written) gets no minimum, and an edit or a preset that waits no rounding
// either: nothing is written for it alone, so it rides the hold already there and ends with it (on
// the schedule, exactly at the next schedule change). Such a timed target is not on the zone; a
// change that goes out is based on the hold the zone shows: with none shown, the target's timed
// hold is ignored, unless it is owed (ours, still to go out: kept), and the change is worked out as
// for a zone on its schedule (its hold then replaces the target's, so a waiting side ends with it).
// False, and the change is refused, when it is timed and the thermostat's clock is not known yet
// (only before its first reply).
bool ZonePauseClimate::hold_for_change_(int asked, Held &out, bool goes_out_now) {
  const uint8_t zone = this->source_->get_zone();
  uint16_t now = 0;
  const bool clock = this->now_mow_(now);
  uint16_t met = this->data_.hold_minutes;
  uint16_t left = 0;
  if (met != 0 && met < HOLD_PERMANENT) {
    left = this->minutes_left_(this->data_.hold_end_mow);
    if (left != LEFT_CLOCK_UNKNOWN && !valid_left_(left))
      met = 0;
  }
  const bool met_timed = met != 0 && met < HOLD_PERMANENT;
  // A change that goes out is based on the hold the zone shows (own writes counted), not on a timed
  // hold the target has and the zone does not (that one was never written). With no hold shown, the
  // zone is on its schedule and the target's timed hold (a waiting value's) is a phantom, ignored:
  // the change is worked out as for any zone on its schedule. An owed one is not a phantom but our
  // own write still to go out (a Hold Minutes just before, held up by the shared gate): like the
  // target's timed hold under a permanent one (a pause's), it is written as the target's own. The
  // kind, not the end, is compared: a write of ours can move a timed hold's end (the hub's round-up).
  uint8_t dummy[2];
  bool permanent = false, timed = false;
  uint16_t shown = 0;
  this->read_real_(dummy, permanent, &shown, &timed);
  const HoldKind view = this->hold_view_(permanent, timed, shown);
  const bool unwritten = clock && goes_out_now && met_timed && view != HOLD_KIND_TIMED;
  const bool phantom = unwritten && view == HOLD_KIND_NONE && !this->data_.owed_hold;
  const uint16_t least = goes_out_now ? MIN_CHANGE_HOLD : 0;
  Held h{met, met_timed ? this->data_.hold_end_mow : HOLD_END_UNKNOWN, false};
  uint16_t minutes = 0;  // nonzero: a new timed hold of this many minutes from now
  if (asked == 0) {
    h = Held{0, HOLD_END_UNKNOWN, met != 0};
  } else if (asked > 0) {
    minutes = infinitesp::InfinitESPComponent::normalize_timed_hold(asked);
    if (minutes < least)
      minutes = least;
    if (met_timed && !unwritten && same_quarter_(left, minutes))
      minutes = 0;  // the target already ends there
  } else if (met == 0 || phantom) {
    uint16_t next = this->minutes_to_next_activity_();
    if (next == 0) {
      next = this->minimum_hold_ != 0 ? this->minimum_hold_ : 60;
      ESP_LOGW(TAG, "Zone %d: schedule not readable; holding %u min instead of until the next schedule period",
               zone, next);
    }
    minutes = next;
    if (goes_out_now) {  // the floors and the hub's grid are for what is written
      if (asked == CHANGE_EDIT && minutes < this->minimum_hold_)
        minutes = this->minimum_hold_;
      if (minutes < least)
        minutes = least;
      minutes = infinitesp::InfinitESPComponent::normalize_timed_hold(minutes);
    }
  } else if (unwritten ||
             (goes_out_now && met_timed && left < MIN_CHANGE_HOLD && !same_quarter_(left, MIN_CHANGE_HOLD))) {
    // Written: the target's end, at least 30 minutes away, on the grid (so it ends with the zone's timer).
    // With `unwritten` this is a pause's timed hold under the permanent one the zone shows, or an owed one.
    minutes = infinitesp::InfinitESPComponent::normalize_timed_hold(left > MIN_CHANGE_HOLD ? left : MIN_CHANGE_HOLD);
  }
  if (minutes != 0)
    h = Held{minutes, end_of_(now, minutes), true};
  if (h.minutes != 0 && h.minutes < HOLD_PERMANENT && !clock) {
    ESP_LOGE(TAG, "Zone %d: change refused: the thermostat's clock is not known yet, so its hold cannot be timed",
             zone);
    return false;
  }
  out = h;
  return true;
}

// A change's hold becomes the target's. A timed one is owed (written) only while the goal also
// owes a setpoint side: it is never written without a side to land. Paused, only the target
// changes; the unpause applies it.
void ZonePauseClimate::store_hold_(const Held &h) {
  if (h.minutes != this->data_.hold_minutes || h.end != this->data_.hold_end_mow)
    this->hold_sent_ = false;
  this->data_.hold_minutes = h.minutes;
  this->data_.hold_end_mow = h.end;
  if (this->data_.paused)
    return;
  this->data_.set_hold = set_hold_of_(h.minutes);
  const bool timed = h.minutes != 0 && h.minutes < HOLD_PERMANENT;
  if (h.owed && (!timed || this->data_.owed_heat || this->data_.owed_cool))
    this->data_.owed_hold = true;
}

// Keeps cool at least SETPOINT_GAP above heat, as the thermostat does: `won` stays and the
// other side moves to exactly the gap. True when the other side moved.
bool ZonePauseClimate::make_valid_(uint8_t &heat, uint8_t &cool, Side won) {
  if (heat + SETPOINT_GAP <= cool)
    return false;
  if (won == HEAT)
    cool = heat + SETPOINT_GAP;
  else
    heat = cool - SETPOINT_GAP;
  return true;
}

// Back to the schedule (Per Schedule, Hold Minutes 0): a temperature still on its way and a
// waiting side are dropped, as the wall does, and the target's hold becomes the schedule.
void ZonePauseClimate::to_schedule_() {
  this->begin_set_goal_();
  this->data_.owed_heat = this->data_.owed_cool = false;
  this->set_desired_(HEAT, 0);
  this->set_desired_(COOL, 0);
  this->store_hold_(Held{0, HOLD_END_UNKNOWN, true});
  this->send_due_ = true;
}

// A waiting target is replaced (a change from outside) or over (its end passed): both waiting
// sides go and Setting Status says why. Silent when nothing waits. A target over whose release
// is still due (it took over a pause restore in OFF) sends the zone to the schedule, as that
// restore alone would. A change from outside (a person's, or the hold ending) drops the due release
// too: it would reload their values away.
void ZonePauseClimate::drop_waiting_(const char *why, bool ran_out) {
  if (!this->waiting_())
    return;
  ESP_LOGI(TAG, "Zone %d: the setting that was waiting (heat %d, cool %d) is dropped: %s", this->source_->get_zone(),
           this->data_.desired_heat, this->data_.desired_cool, why);
  if (!ran_out)
    this->data_.release_due = false;
  else if (this->data_.release_due && this->data_.goal == GOAL_NONE)
    this->to_schedule_();
  this->set_desired_(HEAT, 0);
  this->set_desired_(COOL, 0);
  this->drop_why_ = why;
  this->save_();
  this->publish_all_();
}

// The target is given up: the thermostat's values stand, nothing waits, Setting Status says why.
void ZonePauseClimate::end_target_(const uint8_t real[2], const char *why) {
  this->adopt_target_(HEAT, real[HEAT]);
  this->adopt_target_(COOL, real[COOL]);
  this->set_desired_(HEAT, 0);
  this->set_desired_(COOL, 0);
  this->set_goal_(GOAL_NONE);  // clears every owed part, the preset and the due release
  this->drop_why_ = why;
  this->save_();
  this->publish_all_();
}

// Once a minute, with the thermostat's clock known: a timed target whose end has passed is
// over. Paused, the unpause goes back to the schedule (Setting Status says why); a waiting side
// is dropped. An idle zone follows the thermostat anyway, and a goal in flight is checked when it
// sends.
void ZonePauseClimate::minute_tick_() {
  const uint16_t hold = this->data_.hold_minutes;
  uint16_t now = 0;
  if (hold == 0 || hold >= HOLD_PERMANENT || !this->now_mow_(now))
    return;
  const uint16_t end = this->data_.hold_end_mow;
  if (end != HOLD_END_UNKNOWN && valid_left_(left_(now, end)))
    return;
  if (!this->data_.paused) {
    this->drop_waiting_("the setting's time ran out", true);
    return;
  }
  ESP_LOGI(TAG, "Zone %d: the paused target's hold has ended (%s); the unpause goes back to the schedule",
           this->source_->get_zone(), hold_text_(hold, end));
  this->data_.hold_minutes = 0;
  this->data_.hold_end_mow = HOLD_END_UNKNOWN;
  this->drop_why_ = "the setting's time ran out";
  this->save_();
  this->publish_all_();
}

// The values of the current send chain (issue_send_ empties the list when a chain starts). More
// than the list holds: the newest takes the last slot.
void ZonePauseClimate::add_sent_(Side side, uint8_t value) {
  uint8_t &count = this->sent_count_[side];
  for (uint8_t i = 0; i < count; i++) {
    if (this->sent_[side][i] == value)
      return;
  }
  this->sent_[side][count < sizeof(this->sent_[side]) ? count++ : count - 1] = value;
}

// A value the current send chain carried; it may be showing up late. Ours until SENT_EXCUSE_MS
// after the zone's last write, however long the shared gate held the stages.
bool ZonePauseClimate::is_sent_(Side side, uint8_t value) const {
  if (millis() - this->last_send_ms_ >= SENT_EXCUSE_MS)
    return false;
  for (uint8_t i = 0; i < this->sent_count_[side]; i++) {
    if (this->sent_[side][i] == value)
      return true;
  }
  return false;
}

// A value other than `value` that the current send chain carried for this side and that may still
// show up (is_sent_'s window): the board's own earlier write, not judged yet. When the thermostat
// shows `value`, that write has not landed, and it would be excused as ours if it landed later.
bool ZonePauseClimate::other_sent_(Side side, uint8_t value) const {
  if (millis() - this->last_send_ms_ >= SENT_EXCUSE_MS)
    return false;
  for (uint8_t i = 0; i < this->sent_count_[side]; i++) {
    if (this->sent_[side][i] != value)
      return true;
  }
  return false;
}

// Whether the hold part of the current goal reads back as wanted.
bool ZonePauseClimate::hold_landed_(bool permanent, bool timed, uint16_t hold_minutes) const {
  if (this->data_.goal == GOAL_PAUSE)
    return permanent;
  if (this->data_.goal == GOAL_RESTORE) {
    switch (this->restore_hold_kind_()) {
      case HOLD_KIND_PERMANENT:
        return permanent;
      case HOLD_KIND_TIMED:
        return timed;
      default:
        return !permanent && !timed;
    }
  }
  switch (this->data_.set_hold) {
    case SET_HOLD_PERMANENT:
      return permanent;
    case SET_HOLD_CANCEL:
      return !permanent && !timed;
    case SET_HOLD_TIMED_END: {
      // Not before our write went out (a countdown seen earlier is not ours).
      if (!this->hold_sent_ || !timed || permanent)
        return false;
      // The countdown runs from the moment the thermostat took it; one grid step of slack.
      const uint16_t asked = this->data_.set_hold_minutes;
      return hold_minutes + MIN_TIMED_HOLD >= asked && hold_minutes <= asked + MIN_TIMED_HOLD;
    }
    default:
      return true;
  }
}

// Nothing left for a send to do in the current mode.
bool ZonePauseClimate::satisfied_(const uint8_t real[2], bool permanent, bool timed, uint16_t hold_minutes,
                                  uint8_t fan) const {
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const bool wanted = this->data_.goal == GOAL_PAUSE || this->owed_(side);
    if (lands[s] && wanted && real[s] != this->goal_value_(side))
      return false;
  }
  if (this->data_.goal == GOAL_SET) {
    if (this->data_.owed_fan && fan != this->data_.set_fan)
      return false;
    if (this->data_.owed_hold && !this->hold_landed_(permanent, timed, hold_minutes))
      return false;
    return true;
  }
  if (this->data_.goal == GOAL_PAUSE)
    return permanent;
  return !this->data_.owed_hold;
}

static uint8_t fan_code_(climate::ClimateFanMode m) {
  switch (m) {
    case climate::CLIMATE_FAN_LOW:
      return infinitesp::FAN_LOW;
    case climate::CLIMATE_FAN_MEDIUM:
      return infinitesp::FAN_MED;
    case climate::CLIMATE_FAN_HIGH:
      return infinitesp::FAN_HIGH;
    default:
      return infinitesp::FAN_AUTO;
  }
}

void ZonePauseClimate::control(const climate::ClimateCall &call) {
  this->ensure_started_();
  const uint8_t zone = this->source_->get_zone();
  auto fwd = this->source_->make_call();
  bool forward = false;
  bool changed = false;
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  const bool in_off = !lands[HEAT] && !lands[COOL];

  // The system mode is not affected by pause, but its write is two bus frames of its own, so
  // it waits for the shared gate like every other write. The newest mode asked for wins.
  if (call.get_mode().has_value()) {
    this->pending_mode_ = *call.get_mode();
    this->pending_mode_valid_ = true;
    ESP_LOGD(TAG, "Zone %d: system mode change waiting for a free slot on the bus", zone);
  }

  // The fan: while paused it goes straight through as before; otherwise it is a whole-
  // register write like the setpoints and goes through the gate as part of a SET goal.
  if (call.get_fan_mode().has_value()) {
    if (this->data_.goal == GOAL_PAUSE) {
      fwd.set_fan_mode(*call.get_fan_mode());
      forward = true;
    } else {
      this->begin_set_goal_();
      this->data_.set_fan = fan_code_(*call.get_fan_mode());
      this->data_.owed_fan = true;
      this->drop_why_ = nullptr;
      this->send_due_ = true;
      changed = true;
    }
  }

  // This entity declares two-point targets, so ESPHome removes a lone target_temperature
  // from every call before it gets here; only low and high can arrive.
  const optional<float> low = call.get_target_temperature_low();
  const optional<float> high = call.get_target_temperature_high();
  if (low.has_value() || high.has_value()) {
    // The card shows the target (paused: the snapshot, which the unpause puts back; otherwise
    // what is owed, waiting or real) and sends both sides on every click, so only a side that
    // differs from what it shows is given. NaN is ignored; values clamp to 40-99 F with room
    // for the gap.
    const float asked[2] = {low.has_value() ? *low : NAN, high.has_value() ? *high : NAN};
    const float shown_c[2] = {this->target_temperature_low, this->target_temperature_high};
    const uint8_t lo = this->parent_->celsius_to_setpoint((40.0f - 32.0f) * 5.0f / 9.0f);
    const uint8_t hi = this->parent_->celsius_to_setpoint((99.0f - 32.0f) * 5.0f / 9.0f);
    const uint8_t bottom[2] = {lo, (uint8_t) (lo + SETPOINT_GAP)}, top[2] = {(uint8_t) (hi - SETPOINT_GAP), hi};
    uint8_t shown[2], want[2];
    bool given[2], moved[2] = {false, false};
    for (uint8_t s = 0; s < 2; s++) {
      shown[s] = std::isnan(shown_c[s]) ? (s == HEAT ? this->data_.target_heat : this->data_.target_cool)
                                        : this->parent_->celsius_to_setpoint(shown_c[s]);
      want[s] = shown[s];
      given[s] = !std::isnan(asked[s]);
      if (given[s]) {
        const uint8_t v = this->parent_->celsius_to_setpoint(asked[s]);
        want[s] = v < bottom[s] ? bottom[s] : (v > top[s] ? top[s] : v);
        given[s] = want[s] != shown[s];
      }
    }
    const bool paused = this->data_.paused;
    // A paused target on the schedule is not dismissed: what it shows is from the pause, not
    // what the unpause brings, so the call gives it an edit's hold (below).
    const bool nothing_to_do = !given[HEAT] && !given[COOL] && !(paused && this->data_.hold_minutes == 0);
    // A valid pair: the given side wins (both given: the one that moved further; a tie goes
    // to heat) and the other side moves just enough. A moved side counts as given.
    const uint8_t d_heat = want[HEAT] > shown[HEAT] ? want[HEAT] - shown[HEAT] : shown[HEAT] - want[HEAT];
    const uint8_t d_cool = want[COOL] > shown[COOL] ? want[COOL] - shown[COOL] : shown[COOL] - want[COOL];
    const Side won = given[COOL] && (!given[HEAT] || d_cool > d_heat) ? COOL : HEAT;
    const Side lost = won == HEAT ? COOL : HEAT;
    if (make_valid_(want[HEAT], want[COOL], won))
      moved[lost] = given[lost] = true;
    uint8_t real[2] = {0, 0};
    bool permanent = false;
    this->read_real_(real, permanent);
    // A given side is written when it differs from the thermostat, or when it equals it only because
    // an earlier value of ours for that side has not shown up yet (a tap back to what the thermostat
    // still shows): that one can still land and be excused as ours, so the side stays owed and the
    // latest value is written after it.
    auto to_write = [&](Side side) {
      return want[side] != real[side] || (lands[side] && this->other_sent_(side, want[side]));
    };
    // It goes out now when a write will happen: a side it gives lands in this mode and is to be written,
    // or a side already owed is still on its way. Paused, or with nothing to write, it waits and rides
    // the target's hold (hold_for_change_).
    bool goes_out_now = false;
    for (uint8_t s = 0; s < 2 && !paused; s++) {
      const Side side = static_cast<Side>(s);
      goes_out_now = goes_out_now || (given[s] ? lands[s] && to_write(side) : this->owed_(side));
    }
    Held h{};
    if (nothing_to_do) {
      ESP_LOGD(TAG, "Zone %d: asked %d / %d, what it already shows; nothing to do", zone, want[HEAT], want[COOL]);
    } else if (this->hold_for_change_(CHANGE_EDIT, h, goes_out_now)) {
      this->drop_why_ = nullptr;
      changed = true;
      if (paused) {
        // Into the snapshot, with the edit's hold; nothing is written until the unpause.
        for (uint8_t s = 0; s < 2; s++) {
          if (given[s])
            this->adopt_target_(static_cast<Side>(s), want[s]);
        }
        this->store_hold_(h);
        ESP_LOGI(TAG, "Zone %d is paused: target now %d / %d, %s, applied when the pause ends", zone,
                 this->data_.target_heat, this->data_.target_cool, hold_text_(h.minutes, h.end));
      } else {
        // Never forwarded: the source would send both setpoints from its own cache. A side the
        // mode takes is owed where it differs from the thermostat. One it cannot take (cool in
        // HEAT, either side in OFF) waits for a mode that takes it, unless the thermostat
        // already shows it (then it is in force). A moved side the mode cannot take is owed
        // while the other side goes out now: the thermostat's own gap reaction lands it.
        this->begin_set_goal_();
        for (uint8_t i = 0; i < 2; i++) {
          const Side side = static_cast<Side>(moved[HEAT] ? 1 - i : i);  // a moved side last
          const Side other = side == HEAT ? COOL : HEAT;
          if (!given[side])
            continue;
          this->set_desired_(side, 0);
          if (lands[side] || want[side] == real[side] || (moved[side] && lands[other] && this->owed_(other))) {
            this->adopt_target_(side, want[side]);
            this->set_owed_(side, to_write(side));
            if (want[side] == real[side] && this->owed_(side))
              ESP_LOGI(TAG, "Zone %d: %s back to %d while an earlier value is still on its way; written again after",
                       zone, SIDE_NAMES[side], want[side]);
          } else {
            this->set_owed_(side, false);
            this->set_desired_(side, want[side]);
            ESP_LOGI(TAG, "Zone %d: %s %d kept for when the mode takes it (now %s)", zone, SIDE_NAMES[side],
                     want[side], mode_name_(this->confirmed_mode_));
          }
        }
        this->store_hold_(h);
        if (this->data_.owed_heat || this->data_.owed_cool)
          this->send_due_ = true;
        ESP_LOGI(TAG, "Zone %d: target %d / %d, %s", zone, this->data_.target_heat, this->data_.target_cool,
                 hold_text_(h.minutes, h.end));
      }
    }
  }

  const bool has_preset = call.has_custom_preset() || call.get_preset().has_value();
  if (has_preset) {
    uint8_t activity = NO_PRESET;
    bool hold_perm = false, ignored = false, want_pause = false, hold_timer = false;
    if (call.get_preset().has_value()) {
      switch (*call.get_preset()) {
        case climate::CLIMATE_PRESET_HOME:
          activity = infinitesp::COMFORT_HOME;
          break;
        case climate::CLIMATE_PRESET_AWAY:
          activity = infinitesp::COMFORT_AWAY;
          break;
        case climate::CLIMATE_PRESET_SLEEP:
          activity = infinitesp::COMFORT_SLEEP;
          break;
        default:
          ignored = true;
          break;
      }
    } else {
      auto custom = call.get_custom_preset();
      if (custom == PRESET_PAUSED)
        want_pause = true;
      else if (custom == infinitesp::PRESET_WAKE)
        activity = infinitesp::COMFORT_WAKE;
      else if (custom == infinitesp::PRESET_HOLD_PERM)
        hold_perm = true;
      else if (custom == infinitesp::PRESET_HOLD_TIMED)
        hold_timer = true;
      else if (custom != infinitesp::PRESET_SCHEDULE)
        ignored = true;  // Vacation (Per Schedule is the last branch below: back to the schedule)
    }
    // Picking Paused pauses the zone, exactly as the switch does. Picking any other preset
    // while the zone is paused unpauses it, with this hold: the snapshot values come back
    // and the preset decides the hold. Vacation is a readout: it changes nothing, paused or not.
    if (want_pause) {
      const bool was_paused = this->data_.paused;
      this->request_pause(true);
      if (this->data_.paused && !was_paused)
        ESP_LOGI(TAG, "Zone %d: paused from the card", zone);
    } else {
      const bool unpaused = this->data_.paused && !ignored;
      if (unpaused) {
        // Not sent yet: the preset below replaces this restore's hold in the same action, so
        // the two go out as one send.
        ESP_LOGI(TAG, "Zone %d: UNPAUSE: a preset was picked on the card", zone);
        this->request_pause(false, /*send_now=*/hold_timer);
      }
      if (hold_timer) {
        // Unpaused, it is identical to the switch going off: the snapshot values and the snapshot's
        // own hold. Otherwise a readout, not a command: it must not fall through to the hold branch
        // below, which would read it as "back to the schedule".
        if (!unpaused)
          ESP_LOGI(TAG, "Zone %d: Hold Timer is a readout; set Hold Minutes instead", zone);
      } else if (ignored) {
        // Vacation comes from the bus; InfinitESP takes nothing from Home Assistant for it (it
        // only logs a warning), so there is nothing to forward. The card is republished below.
        ESP_LOGI(TAG, "Zone %d: that preset is a readout (Vacation comes from the bus); nothing changed", zone);
      } else if (activity != NO_PRESET && in_off) {
        // No setpoint lands in this mode (off, or not known yet); an activity means nothing
        // until the system is on.
        ESP_LOGI(TAG, "Zone %d: the system is %s, so the %s preset was not applied", zone,
                 mode_name_(this->confirmed_mode_), ACTIVITY_NAMES[activity]);
      } else if (activity != NO_PRESET) {
        uint8_t heat, cool, fan;
        Held h{};
        if (!this->comfort_setpoints_(activity, heat, cool, fan)) {
          ESP_LOGW(TAG, "Zone %d: no comfort profile for %s yet", zone, ACTIVITY_NAMES[activity]);
        } else if (this->hold_for_change_(CHANGE_PRESET, h)) {
          this->begin_set_goal_();
          uint8_t real[2] = {0, 0};
          bool permanent = false;
          uint8_t real_fan = NO_FAN;
          this->read_real_(real, permanent, nullptr, nullptr, &real_fan);
          // A preset is a "now" gesture: a side the mode cannot take is dropped, not kept.
          for (uint8_t s = 0; s < 2; s++) {
            const Side side = static_cast<Side>(s);
            const uint8_t value = side == HEAT ? heat : cool;
            if (lands[s]) {
              this->adopt_target_(side, value);
              this->set_owed_(side, value != real[s] || this->other_sent_(side, value));
            }
            this->set_desired_(side, 0);
          }
          if (fan != real_fan) {
            this->data_.set_fan = fan;
            this->data_.owed_fan = true;
          }
          this->store_hold_(h);
          this->data_.set_preset = activity;
          this->drop_why_ = nullptr;
          this->send_due_ = true;
          changed = true;
          ESP_LOGI(TAG, "Zone %d: %s: %d / %d, fan %d, %s", zone, ACTIVITY_NAMES[activity], heat, cool, fan,
                   hold_text_(h.minutes, h.end));
        }
      } else {
        // Back to the schedule discards an edit that was still on its way and a value that
        // was waiting for a mode change, as the wall does; holding indefinitely holds what
        // was just set, so that edit stays owed.
        if (hold_perm) {
          this->begin_set_goal_();
          this->store_hold_(Held{HOLD_PERMANENT, HOLD_END_UNKNOWN, true});
          this->send_due_ = true;
        } else {
          this->to_schedule_();
        }
        this->drop_why_ = nullptr;
        changed = true;
        ESP_LOGI(TAG, "Zone %d: %s", zone, hold_perm ? "hold indefinitely (permanent)" : "back to the schedule");
      }
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
  if (this->send_due_ || this->pending_mode_valid_)
    this->evaluate_();
}

void ZonePauseClimate::request_pause(bool pause, bool send_now) {
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
    if (!this->parent_->is_bus_online()) {
      ESP_LOGW(TAG, "Zone %d: pause refused, the Carrier bus is offline (check the board's bus wiring and power)", zone);
      this->publish_all_();
      return;
    }
    if (!this->bus_ready_() || !this->read_real_(real, permanent) || !this->real_is_fresh_()) {
      ESP_LOGW(TAG, "Zone %d: pause refused, no recent answer from the thermostat yet. Try again in a few seconds.",
               zone);
      this->publish_all_();
      return;
    }
    // The snapshot is the target, its hold included. An earlier action that has not finished
    // keeps the remembered value for what it still owes (it may not be on the thermostat
    // yet); everything else is remembered as it is now.
    const bool in_flight = this->data_.goal == GOAL_RESTORE || this->data_.goal == GOAL_SET;
    if (!in_flight || !this->data_.owed_heat)
      this->data_.target_heat = real[HEAT];
    if (!in_flight || !this->data_.owed_cool)
      this->data_.target_cool = real[COOL];
    if (!in_flight && !this->in_quiet_) {
      this->baseline_[HEAT] = real[HEAT];
      this->baseline_[COOL] = real[COOL];
    }
    // A side waiting for a mode is part of the target: it moves into the snapshot.
    for (uint8_t s = 0; s < 2; s++) {
      const Side side = static_cast<Side>(s);
      if (this->desired_(side) != 0) {
        this->adopt_target_(side, this->desired_(side));
        this->set_desired_(side, 0);
      }
    }
    ESP_LOGI(TAG, "Zone %d: PAUSE. Remembering %d / %d, %s%s", zone, this->data_.target_heat, this->data_.target_cool,
             hold_text_(this->data_.hold_minutes, HOLD_END_UNKNOWN), in_flight ? ", some of it still on its way" : "");
    if (this->data_.hold_minutes != 0 && this->data_.hold_minutes < HOLD_PERMANENT &&
        this->data_.hold_end_mow != HOLD_END_UNKNOWN)
      ESP_LOGI(TAG, "Zone %d: its hold runs to %02u:%02u; the unpause puts it back to that time", zone,
               (unsigned) (this->data_.hold_end_mow % 1440 / 60), (unsigned) (this->data_.hold_end_mow % 60));
    this->drop_why_ = nullptr;
    this->data_.wide_heat = this->pause_heat_bus_();
    this->data_.wide_cool = this->pause_cool_bus_();
    this->data_.paused = true;
    this->set_goal_(GOAL_PAUSE);
    this->data_.owed_heat = this->data_.owed_cool = this->data_.owed_hold = this->data_.owed_fan = false;
    this->confirmed_mode_ = this->read_confirmed_mode_();
  } else {
    ESP_LOGI(TAG, "Zone %d: UNPAUSE. Putting back %d / %d, %s", zone, this->data_.target_heat,
             this->data_.target_cool, hold_text_(this->data_.hold_minutes, this->data_.hold_end_mow));
    this->drop_why_ = nullptr;
    this->data_.paused = false;
    this->set_goal_(GOAL_RESTORE);
    this->data_.owed_heat = this->data_.owed_cool = true;
    this->data_.owed_hold = true;
    this->data_.restore_hold = true;
  }
  this->send_due_ = true;
  this->save_();
  this->publish_all_();
  if (send_now)
    this->evaluate_();
}

void ZonePauseClimate::queue_stage_(const Stage &s) {
  if (this->pending_count_ < 2)
    this->pending_[this->pending_count_++] = s;
}

// One bus write. Every stage goes through here, so the shared gate and the quiet period
// always start from the last write.
void ZonePauseClimate::issue_stage_(const Stage &s) {
  const uint8_t zone = this->source_->get_zone();
  const uint32_t now = millis();
  switch (s.kind) {
    case STAGE_HOLD:
      this->parent_->set_zone_hold(zone, s.hold);
      this->own_hold_kind_ = s.hold >= HOLD_PERMANENT ? HOLD_KIND_PERMANENT : (s.hold == 0 ? HOLD_KIND_NONE : HOLD_KIND_TIMED);
      this->own_hold_ms_ = now;
      this->own_hold_valid_ = true;
      this->hold_sent_ = true;
      if (s.hold > 0 && s.hold < HOLD_PERMANENT)  // what hold_landed_ judges: the minutes the hub writes
        this->data_.set_hold_minutes = infinitesp::InfinitESPComponent::normalize_timed_hold(s.hold);
      // A timed hold or a release loads the schedule's values (a permanent hold keeps them).
      if (s.hold < HOLD_PERMANENT)
        this->reload_expected_ = true;
      ESP_LOGI(TAG, "Zone %d: hold write: %u (0 = the schedule, 65535 = permanent)", zone, s.hold);
      break;
    case STAGE_SETPOINTS:
      this->parent_->set_zone_setpoint(zone, s.heat, s.cool);
      this->add_sent_(HEAT, s.heat);
      this->add_sent_(COOL, s.cool);
      ESP_LOGI(TAG, "Zone %d: setpoint write: %d / %d", zone, s.heat, s.cool);
      break;
    case STAGE_FAN:
      this->parent_->set_zone_fan(zone, s.fan);
      ESP_LOGI(TAG, "Zone %d: fan write: %d", zone, s.fan);
      break;
  }
  s_last_write_ms = now;
  s_ever_wrote = true;
  this->last_send_ms_ = now;
  this->in_quiet_ = true;
}

// Builds the send for the goal as it is right now: up to three writes (hold, setpoints,
// fan), the first now and the rest one gap apart. Seen live on 2026-09-21: the thermostat
// drops a zones-register write when another one follows it within a fraction of a second;
// a timed-hold write resets the setpoints to the schedule's values (so it goes first); a
// setpoint write on a scheduled zone makes a permanent hold by itself, and one after a
// release does too (so a release goes after the setpoints).
void ZonePauseClimate::issue_send_(const uint8_t real[2]) {
  const uint8_t zone = this->source_->get_zone();
  uint8_t dummy[2];
  bool permanent = false, timed = false;
  uint16_t real_hold = 0;
  uint8_t real_fan = NO_FAN;
  this->read_real_(dummy, permanent, &real_hold, &timed, &real_fan);
  const HoldKind view = this->hold_view_(permanent, timed, real_hold);
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);

  Stage stages[3];
  uint8_t n = 0;
  bool hold_built = false;
  auto hold_stage = [&](uint16_t minutes) {
    stages[n++] = Stage{STAGE_HOLD, minutes, 0, 0, 0};
    hold_built = true;
  };
  // The goal's setpoints as a valid pair: PAUSE the wide values; otherwise each owed side at
  // its target and the rest as they are, the owed side winning (heat when both or neither).
  auto setpoint_stage = [&]() {
    uint8_t heat = this->data_.wide_heat, cool = this->data_.wide_cool;
    if (this->data_.goal != GOAL_PAUSE) {
      heat = this->data_.owed_heat ? this->data_.target_heat : real[HEAT];
      cool = this->data_.owed_cool ? this->data_.target_cool : real[COOL];
      make_valid_(heat, cool, this->data_.owed_cool && !this->data_.owed_heat ? COOL : HEAT);
    }
    stages[n++] = Stage{STAGE_SETPOINTS, 0, heat, cool, 0};
  };
  // Every permanent write, one rule (seen live): a permanent write over a running
  // timer leaves the countdown frozen under the permanent bit, and the next write turns it
  // back into a timed hold. Over a timed view it is a release, then the setpoints, with no
  // permanent bit of our own (the thermostat then holds them permanently itself); otherwise
  // the setpoints, then the permanent bit if the zone does not show it yet.
  auto permanent_stages = [&](bool setpoints) {
    if (view == HOLD_KIND_TIMED) {
      hold_stage(0);
      if (setpoints)
        setpoint_stage();
      return;
    }
    if (setpoints)
      setpoint_stage();
    if (view != HOLD_KIND_PERMANENT)
      hold_stage(HOLD_PERMANENT);
  };

  if (this->data_.goal == GOAL_PAUSE) {
    permanent_stages(true);
  } else if (this->data_.goal == GOAL_RESTORE) {
    // The unpause is not a change: it puts the target back with its own hold, a timed one
    // with the time left to its end (no 30-minute minimum).
    const bool want_hold = this->data_.restore_hold && this->data_.owed_hold;
    HoldKind kind = want_hold ? this->restore_hold_kind_() : HOLD_KIND_NONE;
    const bool timed_target = this->data_.hold_minutes != 0 && this->data_.hold_minutes < HOLD_PERMANENT;
    uint16_t timed_minutes = 0;
    if (want_hold && timed_target) {
      const uint16_t end = this->data_.hold_end_mow;
      const uint16_t left = this->minutes_left_(end);
      if (left == LEFT_CLOCK_UNKNOWN)
        return;  // waits for the thermostat's clock; the send stays due
      if (valid_left_(left)) {
        timed_minutes = left;  // 1-7 minutes left come back as the hub's 15
        kind = HOLD_KIND_TIMED;
      } else {
        // The end has gone by, or it is not known: the zone goes back to its schedule
        // instead, exactly as a target on the schedule does.
        if (!this->hold_fallback_logged_) {
          this->hold_fallback_logged_ = true;
          ESP_LOGI(TAG, "Zone %d: the hold (%s) has passed or its end is not known; back to the schedule", zone,
                   hold_text_(this->data_.hold_minutes, end));
        }
        this->data_.hold_minutes = 0;
        this->data_.hold_end_mow = HOLD_END_UNKNOWN;
        kind = HOLD_KIND_NONE;
      }
    }
    switch (kind) {
      case HOLD_KIND_PERMANENT:
        permanent_stages(true);
        break;
      case HOLD_KIND_TIMED:
        hold_stage(timed_minutes);
        setpoint_stage();
        break;
      default:
        // On the schedule: values back, then unhold (the thermostat then loads its schedule
        // values, or keeps these; either way the zone is not left wide).
        setpoint_stage();
        if (want_hold && view != HOLD_KIND_NONE)
          hold_stage(0);
        break;
    }
  } else {  // GOAL_SET: the target's hold (set_hold), written only while owed or due
    // A pause restore that was releasing the hold was superseded by this action: the release
    // is still due, so this goal writes its own hold kind whatever the zone shows.
    const bool release_due = this->data_.release_due;
    bool timed_write = false, permanent_write = false, release_write = false;
    uint16_t left = 0;
    if (this->data_.set_hold == SET_HOLD_TIMED_END) {
      left = this->minutes_left_(this->data_.hold_end_mow);
      if (left == LEFT_CLOCK_UNKNOWN)
        return;  // waits for the thermostat's clock; the send stays due
      if (!valid_left_(left) && release_due) {
        // The pause restore (or Per Schedule) this goal joined still owed its release: the end
        // has passed, so the target goes back to the schedule, as that restore alone would.
        ESP_LOGI(TAG, "Zone %d: the hold (%s) has passed; back to the schedule", zone,
                 hold_text_(this->data_.hold_minutes, this->data_.hold_end_mow));
        this->store_hold_(Held{0, HOLD_END_UNKNOWN, true});
      } else if (!valid_left_(left) && (this->data_.owed_heat || this->data_.owed_cool || this->data_.owed_hold)) {
        // A passed or unknown end drops what depended on it; a fan change alone still goes.
        ESP_LOGI(TAG, "Zone %d: the time for this setting ran out before it could be sent; nothing written", zone);
        this->end_target_(real, "the setting's time ran out");
        return;
      }
    }
    if (this->data_.set_hold == SET_HOLD_TIMED_END) {
      // Never a timed hold without an owed side that lands (the sides wait, so it waits too),
      // except one that went out and did not land, once it has been judged (a resend from
      // judge_, or the first send after a judgement in a mode that takes nothing): its write
      // reloads the schedule's values, so the landing sides go out again after it.
      const bool resend = this->resend_count_ > 0 || (this->hold_sent_ && !this->in_quiet_);
      const bool lands_owed = (this->data_.owed_heat && lands[HEAT]) || (this->data_.owed_cool && lands[COOL]);
      timed_write = valid_left_(left) && (lands_owed || resend) &&
                    (release_due || (this->data_.owed_hold && (!this->hold_sent_ || resend)));
    } else if (this->data_.set_hold == SET_HOLD_PERMANENT) {
      permanent_write = release_due || (this->data_.owed_hold && view != HOLD_KIND_PERMANENT);
    } else {
      release_write = release_due || (this->data_.owed_hold && view != HOLD_KIND_NONE);
    }
    if (timed_write || (permanent_write && view == HOLD_KIND_TIMED)) {
      // That write loads the schedule's values: every landing side is owed at its target,
      // even one already there.
      for (uint8_t s = 0; s < 2; s++) {
        if (lands[s])
          this->set_owed_(static_cast<Side>(s), true);
      }
    } else {
      // No reload first: an owed side already at its target is done, unless an earlier value of ours
      // for it may still land (a tap back, see control()): that one stays owed and is written.
      for (uint8_t s = 0; s < 2; s++) {
        const Side side = static_cast<Side>(s);
        if (this->owed_(side) && real[s] == this->goal_value_(side) && !this->other_sent_(side, real[s]))
          this->set_owed_(side, false);
      }
    }
    const bool setpoints = this->data_.owed_heat || this->data_.owed_cool;
    if (timed_write) {
      hold_stage(left);
      if (setpoints)
        setpoint_stage();
    } else if (permanent_write) {
      permanent_stages(setpoints);
    } else {
      if (setpoints)
        setpoint_stage();
      if (release_write)
        hold_stage(0);
    }
    if (hold_built)
      this->data_.release_due = false;  // the hold write carries the release out
    if (this->data_.owed_fan && this->data_.set_fan != real_fan)
      stages[n++] = Stage{STAGE_FAN, 0, 0, 0, this->data_.set_fan};
    else if (this->data_.owed_fan)
      this->data_.owed_fan = false;
  }

  this->send_due_ = false;
  this->pending_count_ = 0;
  if (n == 0) {
    ESP_LOGD(TAG, "Zone %d: nothing to write", zone);
    return;
  }
  this->baseline_[HEAT] = real[HEAT];
  this->baseline_[COOL] = real[COOL];
  this->goal_seen_[HEAT] = this->goal_seen_[COOL] = false;
  if (!this->in_quiet_)  // the last chain was judged: a new one, and its values alone are ours
    this->sent_count_[HEAT] = this->sent_count_[COOL] = 0;
  this->issue_stage_(stages[0]);
  for (uint8_t i = 1; i < n; i++)
    this->queue_stage_(stages[i]);
  if (n > 1)
    ESP_LOGI(TAG, "Zone %d: %d more write%s to follow, %u s apart", zone, n - 1, n > 2 ? "s" : "",
             (unsigned) (SEND_GAP_MS / 1000));
}

// A watched setpoint changed to something that is not ours while the zone was paused: the
// pause is over and that value stands. Nobody tries to work out who did it.
void ZonePauseClimate::end_pause_by_deviation_(const uint8_t real[2], const bool deviated[2]) {
  const uint8_t zone = this->source_->get_zone();
  ESP_LOGW(TAG, "Zone %d: PAUSE CANCELLED: something else changed the setpoints to %d / %d. The zone is no longer "
                "paused and those values stand.",
           zone, real[HEAT], real[COOL]);
  this->data_.paused = false;
  this->drop_why_ = nullptr;
  // Their value is the target from now on, and it wins the pair.
  for (uint8_t s = 0; s < 2; s++) {
    if (deviated[s])
      this->adopt_target_(static_cast<Side>(s), real[s]);
  }
  make_valid_(this->data_.target_heat, this->data_.target_cool, deviated[HEAT] ? HEAT : COOL);
  bool owed[2] = {false, false};
  for (uint8_t s = 0; s < 2; s++) {
    const uint8_t wide = s == HEAT ? this->data_.wide_heat : this->data_.wide_cool;
    const uint8_t target = s == HEAT ? this->data_.target_heat : this->data_.target_cool;
    if (!deviated[s] && real[s] == wide && wide != target)  // still at its wide value: put it back
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

// Judged only outside quiet periods and with no stage pending: once when the quiet period
// ends and then on every evaluation.
void ZonePauseClimate::judge_(const uint8_t real[2], bool permanent, bool timed, uint16_t hold_minutes, uint8_t fan,
                              bool quiet_period_just_ended) {
  const uint8_t zone = this->source_->get_zone();
  bool watched[2], lands[2];
  watched_sides_(this->confirmed_mode_, watched[HEAT], watched[COOL]);
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  const bool in_off = !lands[HEAT] && !lands[COOL];
  bool deviated[2] = {false, false};
  bool changed = false;
  bool restore_ended = false;

  // A target that is the schedule (a restore to the schedule, Per Schedule, Hold Minutes 0):
  // once the thermostat shows no hold, it has loaded its schedule's values, and those are the
  // target. Every owed side is done; nothing is dropped or sent again, and nothing waits.
  const bool schedule_target =
      this->data_.hold_minutes == 0 && ((this->data_.goal == GOAL_RESTORE && this->data_.restore_hold) ||
                                        (this->data_.goal == GOAL_SET && this->data_.set_hold == SET_HOLD_CANCEL));
  if (schedule_target && this->hold_landed_(permanent, timed, hold_minutes)) {
    for (uint8_t s = 0; s < 2; s++) {
      const Side side = static_cast<Side>(s);
      if (this->owed_(side) || this->desired_(side) != 0)
        changed = true;
      this->adopt_target_(side, real[s]);
      this->baseline_[s] = real[s];
      this->set_owed_(side, false);
      this->set_desired_(side, 0);
    }
  }

  // The first judgement after a timed-hold write or a release: the thermostat has loaded its
  // schedule's values, so a side moved to a value that is neither its goal nor one we sent is
  // that reload, not someone else. (Residual: a change made by someone in that window is taken
  // for it.)
  const bool reload = this->reload_expected_ && quiet_period_just_ended;
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const uint8_t value = real[s];
    const uint8_t wide = side == HEAT ? this->data_.wide_heat : this->data_.wide_cool;
    if (value == this->goal_value_(side)) {
      // It landed (or never had to move).
      this->baseline_[s] = value;
      if (this->data_.goal != GOAL_PAUSE && this->owed_(side)) {
        this->set_owed_(side, false);
        changed = true;
        ESP_LOGI(TAG, "Zone %d: %s is at %d", zone, SIDE_NAMES[s], value);
      }
    } else if (value == this->baseline_[s]) {
      // Has not moved: a send that did not land, or a mode that ignores this side.
    } else if (reload && !this->is_sent_(side, value)) {
      this->baseline_[s] = value;
      if (this->data_.goal == GOAL_PAUSE) {
        // Not landed yet (the pause's own resend puts it right); the snapshot keeps its value.
      } else if (lands[s]) {
        this->set_owed_(side, true);  // not landed: sent again once, then given up as always
      } else if (this->data_.goal == GOAL_SET && this->data_.set_preset != NO_PRESET) {
        this->adopt_target_(side, value);  // a preset's other side is the schedule's
      } else {
        this->set_owed_(side, false);
        if (this->desired_(side) == 0)
          this->set_desired_(side, this->goal_value_(side));  // the target's value waits (a waiting one stays)
      }
      changed = true;
      ESP_LOGD(TAG, "Zone %d: %s moved to %d as the thermostat reloaded its schedule", zone, SIDE_NAMES[s], value);
    } else if (this->is_sent_(side, value) || (this->data_.goal == GOAL_RESTORE && value == wide)) {
      // A send of ours showing up late. The next send corrects it.
      this->baseline_[s] = value;
    } else if (this->data_.goal == GOAL_PAUSE && !watched[s]) {
      // The current mode does not use this side: keep the new value as its target (it wins
      // the pair).
      this->baseline_[s] = value;
      this->adopt_target_(side, value);
      make_valid_(this->data_.target_heat, this->data_.target_cool, side);
      changed = true;
      ESP_LOGW(TAG, "Zone %d is paused: %s was changed to %d by something else; kept as the new %s target", zone,
               SIDE_NAMES[s], value, SIDE_NAMES[s]);
    } else if (this->data_.goal == GOAL_PAUSE && in_off && !permanent && !timed && this->last_real_valid_ &&
               this->last_hold_ <= 2 && !quiet_period_just_ended) {
      // Paused in OFF, where nothing is written, and no hold at the last reading or now (a written
      // pause shows its permanent hold): the schedule moved to its next period, or a timer in its
      // last 2 minutes ran out into it. A person's change shows a hold (a wall touch) or removes one
      // (the pause's own included) and still ends the pause. Not at a quiet period's end: the last
      // reading is then newer than the move.
      this->baseline_[s] = value;
      ESP_LOGI(TAG, "Zone %d is paused: %s moved to %d with no hold (the schedule); still paused", zone,
               SIDE_NAMES[s], value);
    } else {
      deviated[s] = true;
      this->baseline_[s] = value;
    }
  }
  if (quiet_period_just_ended)
    this->reload_expected_ = false;

  if (this->data_.goal == GOAL_PAUSE) {
    if (deviated[HEAT] || deviated[COOL]) {
      this->end_pause_by_deviation_(real, deviated);
      return;
    }
  } else {
    // Someone else changed a side: their value stands and wins the pair, no hold of ours is
    // written over it, and a waiting target is dropped whole (people win).
    if (deviated[HEAT] || deviated[COOL]) {
      this->data_.owed_hold = this->data_.release_due = false;
      for (uint8_t s = 0; s < 2; s++) {
        const Side side = static_cast<Side>(s);
        if (!deviated[s])
          continue;
        if (this->owed_(side)) {
          this->set_owed_(side, false);
          ESP_LOGI(TAG, "Zone %d: %s was changed to %d by something else; that value stands", zone, SIDE_NAMES[s],
                   real[s]);
        }
        this->adopt_target_(side, real[s]);
      }
      make_valid_(this->data_.target_heat, this->data_.target_cool, deviated[HEAT] ? HEAT : COOL);
      this->someone_else_();
      changed = true;
    }
    // A side the current mode cannot take stops being owed and becomes that side's desired
    // value, so no goal stays in flight for it. In OFF that applies to a card action (SET)
    // too; a pause restore waits for a mode that takes it.
    if (!in_off || this->data_.goal == GOAL_SET) {
      for (uint8_t s = 0; s < 2; s++) {
        const Side side = static_cast<Side>(s);
        if (this->owed_(side) && !lands[s]) {
          this->set_owed_(side, false);
          if (this->data_.goal == GOAL_RESTORE || this->data_.set_preset == NO_PRESET)
            this->set_desired_(side, this->goal_value_(side));
          changed = true;
          ESP_LOGI(TAG, "Zone %d: %s %d cannot land in %s; kept for when the mode takes it", zone, SIDE_NAMES[s],
                   this->goal_value_(side), mode_name_(this->confirmed_mode_));
        }
      }
    }
    // A timed hold is never owed without an owed side: one not written yet waits with the
    // target. One that went out is judged below and, if it did not land, written again on the
    // next send (issue_send_), in a mode that takes a side.
    if (this->data_.goal == GOAL_SET && this->data_.set_hold == SET_HOLD_TIMED_END && this->data_.owed_hold &&
        !this->hold_sent_ && !this->data_.owed_heat && !this->data_.owed_cool) {
      this->data_.owed_hold = false;
      changed = true;
    }
    if (this->data_.goal == GOAL_SET && this->data_.owed_fan && fan == this->data_.set_fan) {
      this->data_.owed_fan = false;
      changed = true;
    }
    if (this->data_.owed_hold && this->hold_landed_(permanent, timed, hold_minutes)) {
      this->data_.owed_hold = false;
      changed = true;
    }
    if (!this->data_.owed_heat && !this->data_.owed_cool && !this->data_.owed_hold && !this->data_.owed_fan) {
      ESP_LOGI(TAG, "Zone %d: done (%d / %d)", zone, real[HEAT], real[COOL]);
      restore_ended = this->data_.goal == GOAL_RESTORE;
      this->set_goal_(GOAL_NONE);
      changed = true;
    }
  }

  // A wanted value that did not land in a mode that takes it: send again once (MAX_RESENDS), then
  // give up, let the thermostat's own values stand and show them on the card.
  if (quiet_period_just_ended && this->data_.goal != GOAL_NONE &&
      !this->satisfied_(real, permanent, timed, hold_minutes, fan)) {
    if (lands[HEAT] || lands[COOL]) {
      if (this->resend_count_ < MAX_RESENDS) {
        this->resend_count_++;  // a hold still owed goes out again (issue_send_)
        this->send_due_ = true;
        ESP_LOGW(TAG, "Zone %d: the thermostat has not taken everything yet (it shows %d / %d); sending again once",
                 zone, real[HEAT], real[COOL]);
      } else if (this->data_.goal == GOAL_PAUSE) {
        // A pause is not ended here: the zone is still paused and its snapshot must survive.
        // Log once and stop resending; the pause switch is the way out.
        if (this->resend_count_ == MAX_RESENDS) {
          this->resend_count_++;  // log once
          ESP_LOGE(TAG, "Zone %d: the thermostat still shows %d / %d and has not accepted the pause. The zone is "
                        "still marked paused; turn the Pause switch off to release it.",
                   zone, real[HEAT], real[COOL]);
        }
      } else {
        ESP_LOGE(TAG, "Zone %d: gave up: the thermostat did not accept the change. It shows %d / %d (hold: %s); "
                      "those values stand.",
                 zone, real[HEAT], real[COOL], permanent ? "permanent" : (timed ? "timed" : "none"));
        this->end_target_(real, "the thermostat did not accept the change");
        return;
      }
    }
  }

  if (changed) {
    this->save_();
    this->publish_all_();
  }
  // A pause restore that has just finished: a value the confirmed mode now accepts goes out.
  if (restore_ended && !this->data_.paused && (this->data_.desired_heat != 0 || this->data_.desired_cool != 0))
    this->deliver_desired_(real);
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
  bool adopted = false;
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    this->baseline_[s] = real[s];
    if (this->data_.goal == GOAL_NONE)
      continue;
    if (real[s] == wide[s] || real[s] == target[s])
      continue;
    if (this->data_.goal == GOAL_PAUSE && watched[s]) {
      deviated[s] = true;
    } else {
      // Their value is the target from now on, and it wins the pair.
      this->set_owed_(side, false);
      this->adopt_target_(side, real[s]);
      make_valid_(this->data_.target_heat, this->data_.target_cool, side);
      adopted = true;
    }
  }
  if (this->data_.goal == GOAL_PAUSE && (deviated[HEAT] || deviated[COOL])) {
    this->end_pause_by_deviation_(real, deviated);
    return;
  }
  if (adopted && this->data_.goal != GOAL_PAUSE)
    this->someone_else_();
  if (this->data_.goal != GOAL_NONE) {
    ESP_LOGI(TAG, "Zone %d: %s after the restart", zone,
             this->data_.goal == GOAL_PAUSE ? "still paused" : "finishing what was being applied");
    this->send_due_ = true;
  }
  this->save_();
  this->publish_all_();
}

// A waiting side whose mode has arrived goes out with the target's hold: a timed one to its
// end (the hold is written only if the thermostat does not already show that end), no end as
// a permanent one. A side the thermostat already shows is in force and no longer waits. A
// target whose end has passed is dropped; with the clock unknown, delivery waits.
void ZonePauseClimate::deliver_desired_(const uint8_t real[2]) {
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  const uint16_t hold = this->data_.hold_minutes;
  bool hold_owed = false;
  if (hold != 0 && hold < HOLD_PERMANENT) {
    const uint16_t left = this->minutes_left_(this->data_.hold_end_mow);
    if (left == LEFT_CLOCK_UNKNOWN)
      return;
    if (!valid_left_(left)) {
      this->drop_waiting_("the setting's time ran out", true);
      return;
    }
    uint8_t dummy[2];
    bool permanent = false, timed = false;
    uint16_t real_hold = 0;
    this->read_real_(dummy, permanent, &real_hold, &timed);
    hold_owed = !timed || !same_quarter_(real_hold, left);
  }
  const bool in_off = !lands[HEAT] && !lands[COOL];
  bool any = false;
  bool dropped = false;
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const uint8_t value = this->desired_(side);
    if (value == 0)
      continue;
    // Already shown is in force, unless the target's timed hold still has to be written: then
    // a side the mode takes goes out with it, and in a mode that takes neither side (Hold
    // Minutes in OFF) both keep waiting, carrying the hold.
    if (value == real[s] && !(hold_owed && (lands[s] || in_off))) {
      // The thermostat is already there: nothing to send, and no goal for it.
      this->set_desired_(side, 0);
      dropped = true;
      ESP_LOGI(TAG, "Zone %d: %s is already at %d; nothing to send", this->source_->get_zone(), SIDE_NAMES[s], value);
      continue;
    }
    if (!lands[s])
      continue;
    if (!any) {
      this->begin_set_goal_();
      any = true;
    }
    this->adopt_target_(side, value);
    this->set_owed_(side, true);
    this->set_desired_(side, 0);
    ESP_LOGI(TAG, "Zone %d: the mode now takes %s; sending the %d you set earlier", this->source_->get_zone(),
             SIDE_NAMES[s], value);
  }
  if (any) {
    this->store_hold_(Held{hold, this->data_.hold_end_mow, hold_owed});
    this->send_due_ = true;
  }
  if (any || dropped) {
    this->save_();
    this->publish_all_();
  }
}

void ZonePauseClimate::evaluate_() {
  uint8_t real[2] = {0, 0};
  bool permanent = false, timed = false;
  uint16_t hold_minutes = 0;
  uint8_t fan = NO_FAN;
  const bool have = this->read_real_(real, permanent, &hold_minutes, &timed, &fan);
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

  // A system mode change waits for the shared gate like every other write: the hub queues
  // two frames for it and its own retries fit inside the 10 s gap.
  if (this->pending_mode_valid_ && gate_open_() && this->bus_ready_()) {
    auto mode_call = this->source_->make_call();
    mode_call.set_mode(this->pending_mode_);
    // The gate is stamped and the request cleared before the call: performing it makes the
    // source publish, which calls this function again.
    this->pending_mode_valid_ = false;
    s_last_write_ms = millis();
    s_ever_wrote = true;
    ESP_LOGI(TAG, "Zone %d: system mode write sent", this->source_->get_zone());
    mode_call.perform();
  }

  if (!have)
    return;

  // The thermostat's own hold: 0, HOLD_PERMANENT or a countdown (a timed one, at most MAX_COUNTDOWN).
  const bool timed_hold = hold_minutes != 0 && hold_minutes < HOLD_PERMANENT;
  const uint16_t countdown = timed_hold && hold_minutes > MAX_COUNTDOWN ? MAX_COUNTDOWN : hold_minutes;
  // An idle zone's target is what the thermostat holds (the card already mirrors its values):
  // its hold is copied on every evaluation, so any change made elsewhere, a hold-only one
  // included, is the target. Not saved on its own.
  if (!this->data_.paused && this->data_.goal == GOAL_NONE && !this->waiting_()) {
    uint16_t now_mow = 0;
    this->data_.hold_minutes = countdown;
    this->data_.hold_end_mow =
        timed_hold && this->now_mow_(now_mow) ? end_of_(now_mow, countdown) : HOLD_END_UNKNOWN;
    this->data_.release_due = false;  // nothing waits any more: a release that outlived its target goes too
  }

  // A goal that ended inside a send (end_target_) leaves no judgement to end its quiet period.
  // Our own hold write is older than OWN_HOLD_MS by then: forgotten, so millis() wrapping never
  // brings it back.
  if (this->in_quiet_ && this->data_.goal == GOAL_NONE && now - this->last_send_ms_ >= QUIET_MS)
    this->in_quiet_ = this->own_hold_valid_ = false;

  // Nothing of ours in flight and a side moved to anything but its waiting value, or the hold
  // changed: the thermostat's hold is the truth, whoever changed it wins, and the waiting target is
  // dropped whole. That includes the zone's own timed hold running out (the thermostat loads the
  // schedule values), which Setting Status names.
  if (this->data_.goal == GOAL_NONE && this->last_real_valid_ && this->waiting_()) {
    const uint16_t was = this->last_hold_;
    // Only a timed hold in its last 2 minutes ends by itself (as judge_ reads a paused zone in OFF):
    // someone cancelling one with more left, or a permanent one (65535), is a change elsewhere.
    const bool ended = was != 0 && was <= 2 && countdown == 0;
    const char *why = ended ? "the zone's hold ended" : "the zone was changed elsewhere";
    // The log says the same: a hold's own end is not "something else".
    const char *by = ended ? "as the zone's hold ended" : "by something else";
    for (uint8_t s = 0; s < 2; s++) {
      if (real[s] != this->last_real_[s] && real[s] != this->desired_(static_cast<Side>(s))) {
        ESP_LOGI(TAG, "Zone %d: %s changed to %d %s", this->source_->get_zone(), SIDE_NAMES[s], real[s], by);
        this->someone_else_(why);
        break;
      }
    }
    // The hold's kind (none, permanent, timed) changed. Only the kind is compared: a write to this
    // zone can move a timed hold's end (the patched InfinitESP rounds a queued hold up to the next 15
    // minutes), so an end changed alone is not told apart.
    const bool kind = (was == 0) != (countdown == 0) || (was >= HOLD_PERMANENT) != (countdown >= HOLD_PERMANENT);
    if (this->waiting_() && kind) {
      ESP_LOGI(TAG, "Zone %d: its hold changed (%u, now %u) %s", this->source_->get_zone(), was, countdown, by);
      this->someone_else_(why);
    }
  }

  // Judge first, under the mode the values were seen in, so that a change by something
  // else is never folded into a fresh baseline by a send that happens to be due. Nothing is
  // classified while a stage is still queued or the last write is inside its quiet period;
  // landed detection keeps running.
  const bool stages_pending = this->pending_count_ > 0;
  if (this->data_.goal != GOAL_NONE) {
    if (this->in_quiet_ || stages_pending) {
      if (now - this->last_send_ms_ < QUIET_MS || stages_pending) {
        for (uint8_t s = 0; s < 2; s++) {
          if (real[s] == this->goal_value_(static_cast<Side>(s)))
            this->goal_seen_[s] = true;
        }
      } else {
        this->in_quiet_ = this->own_hold_valid_ = false;  // as above: our hold write is forgotten
        // A side that reached the goal value counts from there, so a change made after our
        // write landed is still a change.
        for (uint8_t s = 0; s < 2; s++) {
          if (this->goal_seen_[s])
            this->baseline_[s] = this->goal_value_(static_cast<Side>(s));
        }
        this->judge_(real, permanent, timed, hold_minutes, fan, /*quiet_period_just_ended=*/true);
      }
    } else {
      this->judge_(real, permanent, timed, hold_minutes, fan, /*quiet_period_just_ended=*/false);
    }
  }
  // Kept after judging: the idle watch above and judge_ both compare against the previous reading.
  this->last_real_[HEAT] = real[HEAT];
  this->last_real_[COOL] = real[COOL];
  this->last_hold_ = countdown;
  this->last_real_valid_ = true;

  // The one place a system mode change is taken, after judging so that values are judged
  // under the mode they were seen in. Anything still in flight is sent again under the new
  // mode; a value that was waiting for a mode that takes it is delivered.
  const bool mode_changed = mode != MODE_UNKNOWN && mode != this->confirmed_mode_;
  if (mode_changed) {
    const bool in_flight = this->data_.goal != GOAL_NONE;
    ESP_LOGI(TAG, "Zone %d: system mode changed from %s to %s%s", this->source_->get_zone(),
             mode_name_(this->confirmed_mode_), mode_name_(mode), in_flight ? ", sending again" : "");
    this->confirmed_mode_ = mode;
    if (in_flight) {
      this->send_due_ = true;
      this->resend_count_ = 0;
      this->pending_count_ = 0;
    }
  }
  // A pause restore is left alone: it already carries what the zone must get back. A pending
  // value is delivered when that restore ends.
  const bool deliver_now = this->data_.goal == GOAL_NONE || (mode_changed && this->data_.goal == GOAL_SET);
  if (deliver_now && mode != MODE_UNKNOWN && (this->data_.desired_heat != 0 || this->data_.desired_cool != 0))
    this->deliver_desired_(real);
  if (this->data_.goal == GOAL_NONE)
    return;

  if (!gate_open_() || !this->bus_ready_())
    return;
  if (this->pending_count_ > 0 && !this->send_due_) {
    const Stage next = this->pending_[0];
    for (uint8_t i = 1; i < this->pending_count_; i++)
      this->pending_[i - 1] = this->pending_[i];
    this->pending_count_--;
    this->issue_stage_(next);
    return;
  }
  if (!this->send_due_)
    return;
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  // Nothing is sent into a mode where nothing lands; the send stays due.
  if (!lands[HEAT] && !lands[COOL])
    return;
  if (this->data_.goal != GOAL_SET && this->satisfied_(real, permanent, timed, hold_minutes, fan) &&
      !this->in_quiet_) {
    this->send_due_ = false;  // already where it should be; nothing to send
    return;
  }
  if (this->real_is_fresh_())
    this->issue_send_(real);  // recomputes the whole send; pending stages are superseded
}

// ---- schedule and comfort profiles --------------------------------------------------

bool ZonePauseClimate::bus_clock_(uint8_t &weekday, uint16_t &minutes) const {
  const auto *state = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT, infinitesp::REG_SAM_STATE);
  if (state == nullptr || state->size() <= infinitesp::REG3B02_MINUTES + 1)
    return false;
  weekday = (*state)[infinitesp::REG3B02_WEEKDAY];
  minutes = ((uint16_t) (*state)[infinitesp::REG3B02_MINUTES] << 8) | (*state)[infinitesp::REG3B02_MINUTES + 1];
  return weekday < 7 && minutes < 1440;
}

// The zone's schedule row (0x4002 + zone-1, stored under the thermostat's address) when a read
// of it is fresh and whole; nullptr otherwise.
const std::vector<uint8_t> *ZonePauseClimate::schedule_row_() const {
  if (!this->schedule_read_valid_ || millis() - this->schedule_read_ms_ > SCHEDULE_STALE_MS)
    return nullptr;
  const auto *row = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT,
                                                (uint16_t) (infinitesp::REG_TSTAT_SCHEDULE + this->source_->get_zone() - 1));
  return row != nullptr && row->size() >= 70 ? row : nullptr;
}

// Verified on this thermostat (2026-09-22): 7 days from Sunday x 5 periods x (start minute
// / 15, activity); a disabled period has start 0x60 (24:00). Bus clock: weekday 0 = Sunday.
uint8_t ZonePauseClimate::schedule_activity_now_() const {
  uint8_t weekday;
  uint16_t minutes;
  const auto *row = this->schedule_row_();
  if (row == nullptr || !this->bus_clock_(weekday, minutes))
    return NO_PRESET;
  // The period in force: the latest enabled start today at or before now, else the last
  // enabled period of the previous day.
  uint8_t activity = NO_PRESET;
  for (int d = 0; d < 2 && activity == NO_PRESET; d++) {
    const uint8_t day = (weekday + 7 - d) % 7;
    int best = -1;
    for (uint8_t p = 0; p < 5; p++) {
      const uint8_t start = (*row)[day * 10 + p * 2];
      if (start >= 96)
        continue;
      const uint16_t start_min = start * 15;
      if (d == 0 && start_min > minutes)
        continue;
      if (best < 0 || start_min >= (*row)[day * 10 + best * 2] * 15)
        best = p;
    }
    if (best >= 0)
      activity = (*row)[day * 10 + best * 2 + 1];
  }
  return activity < 5 ? activity : NO_PRESET;
}

// Minutes from now to the next enabled period start (today, else the next days), 0 when
// the schedule cannot be read.
uint16_t ZonePauseClimate::minutes_to_next_activity_() const {
  uint8_t weekday;
  uint16_t minutes;
  const auto *row = this->schedule_row_();
  if (row == nullptr || !this->bus_clock_(weekday, minutes))
    return 0;
  for (int d = 0; d < 7; d++) {
    const uint8_t day = (weekday + d) % 7;
    uint16_t best = 0xFFFF;
    for (uint8_t p = 0; p < 5; p++) {
      const uint8_t start = (*row)[day * 10 + p * 2];
      if (start >= 96)
        continue;
      const uint16_t start_min = start * 15;
      if (d == 0 && start_min <= minutes)
        continue;
      if (start_min < best)
        best = start_min;
    }
    if (best != 0xFFFF) {
      const uint32_t delta = (uint32_t) d * 1440 + best - minutes;
      return delta > MAX_WRITTEN_HOLD ? MAX_WRITTEN_HOLD : (uint16_t) delta;
    }
  }
  return 0;
}

// The activity's setpoints (bus units) and fan from the zone's comfort row, which the hub
// keeps refreshed (400A + zone-1, stored under the thermostat address).
bool ZonePauseClimate::comfort_setpoints_(uint8_t activity, uint8_t &heat, uint8_t &cool, uint8_t &fan) const {
  const auto *comfort = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT,
                                                    infinitesp::comfort_reg_for_zone(this->source_->get_zone()));
  if (comfort == nullptr || comfort->size() < (size_t) (activity + 1) * infinitesp::COMFORT_ENTRY_SIZE)
    return false;
  const uint8_t base = activity * infinitesp::COMFORT_ENTRY_SIZE;
  heat = this->parent_->celsius_to_setpoint(this->parent_->comfort_byte_to_celsius((*comfort)[base + 0]));
  cool = this->parent_->celsius_to_setpoint(this->parent_->comfort_byte_to_celsius((*comfort)[base + 1]));
  fan = (*comfort)[base + 2];
  return heat != 0 && cool != 0;
}

// ---- what the card shows -----------------------------------------------------------------

void ZonePauseClimate::mirror_from_source_() {
  this->current_temperature = this->source_->current_temperature;
  this->mode = this->source_->mode;
  // A mode change waiting for the gate is shown at once, so the card does not snap back.
  if (this->pending_mode_valid_)
    this->mode = this->pending_mode_;
  this->action = this->source_->action;
  this->fan_mode = this->source_->fan_mode;

  // Targets: the snapshot while paused; otherwise each side shows what is owed or desired
  // for it, else the real value.
  const bool show_heat = this->data_.goal == GOAL_PAUSE || this->data_.owed_heat || this->data_.desired_heat != 0;
  const bool show_cool = this->data_.goal == GOAL_PAUSE || this->data_.owed_cool || this->data_.desired_cool != 0;
  const uint8_t heat = this->data_.goal != GOAL_PAUSE && this->data_.desired_heat != 0 && !this->data_.owed_heat
                           ? this->data_.desired_heat
                           : this->data_.target_heat;
  const uint8_t cool = this->data_.goal != GOAL_PAUSE && this->data_.desired_cool != 0 && !this->data_.owed_cool
                           ? this->data_.desired_cool
                           : this->data_.target_cool;
  this->target_temperature_low =
      show_heat ? this->parent_->setpoint_to_celsius(heat) : this->source_->target_temperature_low;
  this->target_temperature_high =
      show_cool ? this->parent_->setpoint_to_celsius(cool) : this->source_->target_temperature_high;
  this->infer_preset_();
}

// The preset field: the activity whose settings the zone really holds (on the sides the
// mode governs, plus the fan; the manual row excluded), the schedule breaking a tie when
// there is no hold; otherwise the hold kind. Judged from the thermostat's own replies.
void ZonePauseClimate::infer_preset_() {
  // A paused zone says so on the card. The pause switch is unchanged and stays the handle
  // automations use.
  if (this->data_.paused) {
    this->set_custom_preset_(PRESET_PAUSED);
    return;
  }
  if (this->data_.goal == GOAL_PAUSE || this->data_.goal == GOAL_RESTORE) {
    this->clear_custom_preset_();
    this->preset.reset();
    return;
  }
  if (this->data_.goal == GOAL_SET && this->data_.set_preset != NO_PRESET) {
    this->show_activity_(this->data_.set_preset);  // the activity chosen, while it is being applied
    return;
  }
  if (this->source_->has_custom_preset() && this->source_->get_custom_preset() == infinitesp::PRESET_VACATION) {
    this->set_custom_preset_(infinitesp::PRESET_VACATION);
    return;
  }
  uint8_t real[2] = {0, 0};
  bool permanent = false, timed = false;
  uint16_t hold_minutes = 0;
  uint8_t fan = NO_FAN;
  if (!this->read_real_(real, permanent, &hold_minutes, &timed, &fan)) {
    this->clear_custom_preset_();
    this->preset.reset();
    return;
  }
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  const bool govern[2] = {lands[HEAT] || !lands[COOL], lands[COOL] || !lands[HEAT]};  // both when neither lands
  uint8_t matches = 0, matched = NO_PRESET;
  uint8_t candidates[4];
  for (uint8_t a = 0; a < 4; a++) {
    uint8_t h, c, f;
    if (!this->comfort_setpoints_(a, h, c, f))
      continue;
    if ((govern[HEAT] && h != real[HEAT]) || (govern[COOL] && c != real[COOL]) || f != fan)
      continue;
    candidates[matches++] = a;
    matched = a;
  }
  const bool held = permanent || timed;
  if (matches > 1) {
    matched = NO_PRESET;
    if (!held) {
      const uint8_t now_act = this->schedule_activity_now_();
      for (uint8_t i = 0; i < matches; i++)
        if (candidates[i] == now_act)
          matched = now_act;
      if (matched == NO_PRESET)
        matched = candidates[0];
    }
  }
  if (matched != NO_PRESET)
    this->show_activity_(matched);
  else if (permanent)
    this->set_custom_preset_(infinitesp::PRESET_HOLD_PERM);
  else if (timed)
    this->set_custom_preset_(infinitesp::PRESET_HOLD_TIMED);
  else
    this->set_custom_preset_(infinitesp::PRESET_SCHEDULE);
}

// The card's preset for a comfort activity: home, away and sleep are standard presets, wake a custom one.
void ZonePauseClimate::show_activity_(uint8_t activity) {
  switch (activity) {
    case infinitesp::COMFORT_HOME:
      this->set_preset_(climate::CLIMATE_PRESET_HOME);
      break;
    case infinitesp::COMFORT_AWAY:
      this->set_preset_(climate::CLIMATE_PRESET_AWAY);
      break;
    case infinitesp::COMFORT_SLEEP:
      this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
      break;
    default:
      this->set_custom_preset_(infinitesp::PRESET_WAKE);
      break;
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

// Hold Minutes shows the target's minutes left (idle, that is the thermostat's countdown); 0
// for the schedule and for a hold with no end, as InfinitESP's own does (Hold State tells
// them apart).
void ZonePauseClimate::publish_hold_minutes_() {
  if (this->hold_minutes_number_ == nullptr || !this->started_)
    return;
  const uint16_t hold = this->data_.hold_minutes;
  const uint16_t left = hold != 0 && hold < HOLD_PERMANENT ? this->minutes_left_(this->data_.hold_end_mow) : 0;
  const float value = valid_left_(left) ? (float) left : 0.0f;
  if (!this->hold_minutes_number_->has_state() || this->hold_minutes_number_->state != value)
    this->hold_minutes_number_->publish_state(value);
}

// Hold Until shows where Hold Minutes' minutes left end, as a time of day: shown exactly when Hold
// Minutes shows more than 0, unknown when it shows 0 (the schedule, a hold with no end, an end
// that has passed, or the thermostat's clock not known). Refreshed wherever Hold Minutes is.
void ZonePauseClimate::publish_hold_until_() {
#ifdef USE_DATETIME_TIME
  if (this->hold_until_time_ == nullptr || !this->started_)
    return;
  const uint16_t hold = this->data_.hold_minutes;
  const uint16_t left = hold != 0 && hold < HOLD_PERMANENT ? this->minutes_left_(this->data_.hold_end_mow) : 0;
  this->hold_until_time_->show(valid_left_(left) ? this->data_.hold_end_mow % 1440 : -1);
#endif
}

void ZonePauseClimate::publish_all_() {
  this->mirror_from_source_();
  this->publish_state();
  this->publish_hold_minutes_();
  this->publish_hold_until_();
  if (this->pause_switch_ != nullptr &&
      (!this->switch_published_ || this->pause_switch_->state != this->data_.paused)) {
    this->switch_published_ = true;
    this->pause_switch_->publish_state(this->data_.paused);
  }
  // Setting Status: what the target is waiting for, or why the last one was dropped (paused too:
  // its hold ended during the pause). Nothing waits outside the target while paused.
  if (this->setting_status_sensor_ != nullptr) {
    const char *status = "Nothing waiting";
    std::string dropped;  // built only for "Dropped: <why>"
    bool lands[2];
    landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
    if (this->data_.paused && this->drop_why_ == nullptr) {
      status = "Waiting for the pause to end";
    } else if (this->waiting_() && !lands[HEAT] && !lands[COOL]) {
      status = "Waiting for the system to turn on";
    } else if (this->waiting_()) {
      status = this->data_.desired_heat != 0 ? "Waiting for heat or auto mode" : "Waiting for cool or auto mode";
    } else if (this->drop_why_ != nullptr) {
      dropped = std::string("Dropped: ") + this->drop_why_;
      status = dropped.c_str();
    }
    if (!this->setting_status_sensor_->has_state() || this->setting_status_sensor_->state != status)
      this->setting_status_sensor_->publish_state(status);
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
