#include "zone_pause.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace zone_pause {

static const char *const TAG = "zone_pause";
static const uint8_t SAVED_VERSION = 5;
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
// A value we sent can show up late (the gap, the quiet period, then some). After this long
// a real value equal to something we once sent is somebody else's doing.
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
static const uint16_t MAX_TIMED_HOLD = 1425;
// A desired value the mode could not take is forgotten after this long.
static const uint16_t DESIRED_MAX_AGE_MIN = 1440;
// Schedule row: read at boot (staggered per zone) and then this often; stale after twice that.
static const uint32_t SCHEDULE_REFRESH_MS = 6UL * 3600UL * 1000UL;
static const uint32_t SCHEDULE_STALE_MS = 12UL * 3600UL * 1000UL;

static const uint8_t MODE_UNKNOWN = 0xFF;
static const uint16_t HOLD_PERMANENT = infinitesp::InfinitESPComponent::HOLD_PERMANENT;
// A timed snapshot hold is kept as the minute of the week it ends on (Sunday 00:00 = 0).
static const uint16_t MINUTES_PER_WEEK = 7 * 1440;
static const uint16_t HOLD_END_UNKNOWN = 0xFFFF;
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
                    "set its temperatures by hand.",
               this->source_->get_zone());
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
  while (this->minute_accum_ms_ >= 60000) {
    this->minute_accum_ms_ -= 60000;
    if (this->data_.desired_heat != 0 && this->data_.desired_heat_age < 0xFFFF)
      this->data_.desired_heat_age++;
    if (this->data_.desired_cool != 0 && this->data_.desired_cool_age < 0xFFFF)
      this->data_.desired_cool_age++;
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

void ZonePauseClimate::set_desired_(Side side, uint8_t value) {
  if (side == HEAT) {
    this->data_.desired_heat = value;
    this->data_.desired_heat_age = 0;
  } else {
    this->data_.desired_cool = value;
    this->data_.desired_cool_age = 0;
  }
}

// The kind of hold a pause restore puts back (plan 3.5). A timed hold is remembered by the
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
  // A pause takes its own hold, so a release owed by something else must not leak past it.
  if (goal == GOAL_PAUSE)
    this->data_.release_due = false;
  if (goal == GOAL_NONE) {
    this->send_due_ = false;
    this->sent_count_[HEAT] = this->sent_count_[COOL] = 0;
    this->data_.owed_heat = this->data_.owed_cool = this->data_.owed_hold = this->data_.owed_fan = false;
    this->data_.restore_hold = false;
    this->data_.set_fan = NO_FAN;
    this->data_.set_hold = SET_HOLD_KEEP;
    this->data_.set_hold_minutes = 0;
    this->data_.set_preset = NO_PRESET;
    this->data_.release_due = false;
  }
}

// A card action (edit, preset, hold command) starts or joins a SET goal. Coming from NONE
// the targets start as the real values, so an untouched side is sent as it is. Coming from a
// RESTORE, what the restore still owed is kept and the person's action wins over its hold.
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
    // The new action brings its own hold and fan; it never inherits the old one's. The owed
    // setpoint sides stay (a bump after Home keeps Home's other side, as the wall would).
    this->data_.set_hold = SET_HOLD_KEEP;
    this->data_.set_hold_minutes = 0;
    this->data_.owed_hold = false;
    this->data_.owed_fan = false;
    this->data_.set_fan = NO_FAN;
    this->resend_count_ = 0;
    this->hold_fallback_logged_ = false;
    return;
  }
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
  // From RESTORE: keep the owed sides and their targets; the hold part is dropped. Unless
  // that restore was putting a permanent hold back, the hold the zone shows (a pause holds it
  // permanently) is not the hold anybody wants, so a release stays due and the next hold
  // write of this goal carries it out.
  if (this->data_.goal == GOAL_RESTORE && this->data_.restore_hold && this->data_.owed_hold &&
      this->restore_hold_kind_() != HOLD_KIND_PERMANENT)
    this->data_.release_due = true;
  this->set_goal_(GOAL_SET);
  this->data_.owed_hold = false;
  this->data_.restore_hold = false;
  this->data_.owed_fan = false;
  this->data_.set_fan = NO_FAN;
  this->data_.set_hold = SET_HOLD_KEEP;
  this->data_.set_hold_minutes = 0;
  this->data_.set_preset = NO_PRESET;
  this->data_.target_changed = false;
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
    case SET_HOLD_TIMED_NEXT:
    case SET_HOLD_TIMED_MIN: {
      if (!timed || permanent)
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
      this->send_due_ = true;
      changed = true;
    }
  }

  // This entity declares two-point targets, so ESPHome removes a lone target_temperature
  // from every call before it gets here; only low and high can arrive.
  const optional<float> low = call.get_target_temperature_low();
  const optional<float> high = call.get_target_temperature_high();
  if (low.has_value() || high.has_value()) {
    const uint8_t heat = low.has_value() ? this->parent_->celsius_to_setpoint(*low) : this->data_.target_heat;
    const uint8_t cool = high.has_value() ? this->parent_->celsius_to_setpoint(*high) : this->data_.target_cool;
    if (this->data_.goal == GOAL_PAUSE) {
      // Remembered, applied when the pause ends (pause rules unchanged). The pair is taken as
      // given; the thermostat widens the heat/cool gap itself when it is written.
      this->data_.target_heat = heat;
      this->data_.target_cool = cool;
      this->data_.target_changed = true;
      changed = true;
      ESP_LOGI(TAG, "Zone %d is paused: target now %d / %d, applied when the pause ends", zone, heat, cool);
    } else {
      // Home Assistant's card sends both sides on every click, so only a side that differs
      // from what the card shows is the person's edit.
      const float shown_low = this->target_temperature_low;
      const float shown_high = this->target_temperature_high;
      bool given[2];
      given[HEAT] = low.has_value() && (std::isnan(shown_low) || heat != this->parent_->celsius_to_setpoint(shown_low));
      given[COOL] =
          high.has_value() && (std::isnan(shown_high) || cool != this->parent_->celsius_to_setpoint(shown_high));
      if (!given[HEAT] && !given[COOL]) {
        ESP_LOGD(TAG, "Zone %d: card sent %d / %d, the values it already shows; nothing to do", zone, heat, cool);
      } else {
        // Never forwarded: the source would send both setpoints from its own cache. The proxy
        // owns the edit: a side the mode takes is sent and owed; a side it cannot take (the
        // cool side in HEAT, either side in OFF) is kept as the desired value and delivered
        // when the mode changes.
        this->begin_set_goal_();
        bool desired_set = false;
        for (uint8_t s = 0; s < 2; s++) {
          const Side side = static_cast<Side>(s);
          if (!given[s])
            continue;
          const uint8_t value = side == HEAT ? heat : cool;
          if (lands[s]) {
            this->adopt_target_(side, value);
            this->set_owed_(side, true);
            this->set_desired_(side, 0);
          } else {
            this->set_desired_(side, value);
            desired_set = true;
            ESP_LOGI(TAG, "Zone %d: %s %d kept for when the mode takes it (now %s)", zone, SIDE_NAMES[s], value,
                     mode_name_(this->confirmed_mode_));
          }
        }
        // A side that only became a desired value is not a goal in flight: it arms no hold
        // and writes nothing. Only an owed side does.
        const bool owed_now = this->data_.owed_heat || this->data_.owed_cool;
        if (!owed_now && !desired_set) {
          ESP_LOGD(TAG, "Zone %d: card sent %d / %d, nothing for the thermostat to do", zone, heat, cool);
        } else {
          if (owed_now) {
            if (this->data_.set_hold == SET_HOLD_KEEP)
              this->data_.set_hold = SET_HOLD_TIMED_MIN;  // decided against the hold state at send time
            this->send_due_ = true;
          }
          changed = true;
          ESP_LOGI(TAG, "Zone %d: target %d / %d", zone, this->data_.target_heat, this->data_.target_cool);
        }
      }
    }
  }

  const bool has_preset = call.has_custom_preset() || call.get_preset().has_value();
  if (has_preset) {
    uint8_t activity = NO_PRESET;
    bool hold_perm = false, hold_cancel = false, pass = false, want_pause = false, hold_timer = false;
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
          pass = true;
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
      else if (custom == infinitesp::PRESET_SCHEDULE)
        hold_cancel = true;
      else if (custom == infinitesp::PRESET_HOLD_TIMED)
        hold_timer = true;
      else
        pass = true;  // Vacation: InfinitESP handles it
    }
    // Picking Paused pauses the zone, exactly as the switch does. Picking any other preset
    // while the zone is paused means "resume, with this hold": the snapshot values come back
    // and the preset decides the hold. Vacation is forwarded whether or not the zone is
    // paused, as it always was.
    bool resumed = false;
    if (want_pause) {
      const bool was_paused = this->data_.paused;
      this->request_pause(true);
      if (this->data_.paused && !was_paused)
        ESP_LOGI(TAG, "Zone %d: paused from the card", zone);
    } else {
      if (this->data_.paused && !pass) {
        // Not sent yet: the preset below replaces this restore's hold in the same action, so
        // the two go out as one send.
        this->request_pause(false, /*send_now=*/hold_timer);
        resumed = true;
      }
      if (hold_timer) {
        if (resumed) {
          // Identical to the switch going off: the snapshot values and the snapshot's own hold.
          ESP_LOGI(TAG, "Zone %d: resumed from the card", zone);
        } else {
          // A readout, not a command: it must not fall through to the hold branch below,
          // which would read it as "back to the schedule".
          ESP_LOGI(TAG, "Zone %d: Hold Timer is a readout; set Hold Minutes or Hold Until instead", zone);
        }
      } else if (pass) {
        if (call.has_custom_preset()) {
          auto custom = call.get_custom_preset();
          fwd.set_preset(custom.c_str(), custom.size());
        } else {
          fwd.set_preset(*call.get_preset());
        }
        forward = true;
      } else if (activity != NO_PRESET && in_off) {
        // No setpoint lands in this mode (off, or not known yet); an activity means nothing
        // until the system is on.
        ESP_LOGI(TAG, "Zone %d: nothing lands in %s; %s not applied%s", zone, mode_name_(this->confirmed_mode_),
                 ACTIVITY_NAMES[activity], resumed ? " (zone resumed)" : "");
      } else if (activity != NO_PRESET) {
        uint8_t heat, cool, fan;
        if (!this->comfort_setpoints_(activity, heat, cool, fan)) {
          ESP_LOGW(TAG, "Zone %d: no comfort profile for %s yet%s", zone, ACTIVITY_NAMES[activity],
                   resumed ? " (zone resumed)" : "");
        } else {
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
              this->set_owed_(side, value != real[s]);
            }
            this->set_desired_(side, 0);
          }
          if (fan != real_fan) {
            this->data_.set_fan = fan;
            this->data_.owed_fan = true;
          }
          this->data_.set_hold = SET_HOLD_TIMED_NEXT;
          this->data_.set_preset = activity;
          this->send_due_ = true;
          changed = true;
          ESP_LOGI(TAG, "Zone %d: %s: %d / %d, fan %d, until the next scheduled activity%s", zone,
                   ACTIVITY_NAMES[activity], heat, cool, fan, resumed ? " (resumed from the card)" : "");
        }
      } else {
        this->begin_set_goal_();
        this->data_.set_hold = hold_perm ? SET_HOLD_PERMANENT : SET_HOLD_CANCEL;
        this->data_.owed_hold = true;
        if (!hold_perm) {
          // Back to the schedule discards an edit that was still on its way and a value that
          // was waiting for a mode change, as the wall does; holding indefinitely holds what
          // was just set, so that edit stays owed.
          this->data_.owed_heat = this->data_.owed_cool = false;
          this->set_desired_(HEAT, 0);
          this->set_desired_(COOL, 0);
        }
        this->send_due_ = true;
        changed = true;
        ESP_LOGI(TAG, "Zone %d: %s%s", zone, resumed ? "resumed from the card, " : "",
                 hold_perm ? "hold indefinitely (permanent)" : "back to the schedule");
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
  // A timed hold is remembered by the minute of the week it ends on, from the thermostat's
  // own clock, so the restore puts back that same end time however long the pause lasts.
  auto hold_end_of = [this](uint16_t hold) -> uint16_t {
    uint8_t weekday = 0;
    uint16_t now_min = 0;
    if (hold == 0 || hold >= HOLD_PERMANENT || !this->bus_clock_(weekday, now_min))
      return HOLD_END_UNKNOWN;
    return (uint16_t) (((uint32_t) weekday * 1440 + now_min + hold) % MINUTES_PER_WEEK);
  };

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
    if (this->data_.goal == GOAL_RESTORE || this->data_.goal == GOAL_SET) {
      // An earlier action has not finished. What is still owed may not be on the thermostat
      // yet, so keep the remembered value for it; everything else is remembered as it is now.
      if (!this->data_.owed_heat)
        this->data_.target_heat = real[HEAT];
      if (!this->data_.owed_cool)
        this->data_.target_cool = real[COOL];
      // A hold this component still owes is not on the thermostat yet, so the hold the
      // earlier snapshot remembered (and its end time) stands.
      if (this->data_.goal == GOAL_SET || !this->data_.owed_hold) {
        this->data_.hold_minutes = hold_minutes;
        this->data_.hold_end_mow = hold_end_of(hold_minutes);
      }
      ESP_LOGI(TAG, "Zone %d: PAUSE. Remembering %d / %d (hold %u), some of it still on its way", zone,
               this->data_.target_heat, this->data_.target_cool, this->data_.hold_minutes);
    } else {
      this->data_.target_heat = real[HEAT];
      this->data_.target_cool = real[COOL];
      this->data_.hold_minutes = hold_minutes;
      this->data_.hold_end_mow = hold_end_of(hold_minutes);
      if (!this->in_quiet_) {
        this->baseline_[HEAT] = real[HEAT];
        this->baseline_[COOL] = real[COOL];
      }
      ESP_LOGI(TAG, "Zone %d: PAUSE. Remembering %d / %d (hold %u)", zone, real[HEAT], real[COOL], hold_minutes);
    }
    if (this->data_.hold_end_mow != HOLD_END_UNKNOWN)
      ESP_LOGI(TAG, "Zone %d: its hold runs to %02u:%02u; the unpause puts it back to that time", zone,
               (unsigned) (this->data_.hold_end_mow % 1440 / 60), (unsigned) (this->data_.hold_end_mow % 60));
    this->data_.target_changed = false;
    this->data_.wide_heat = this->pause_heat_bus_();
    this->data_.wide_cool = this->pause_cool_bus_();
    this->data_.paused = true;
    this->set_goal_(GOAL_PAUSE);
    this->data_.owed_heat = this->data_.owed_cool = this->data_.owed_hold = this->data_.owed_fan = false;
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
      ESP_LOGI(TAG, "Zone %d: hold write: %s", zone,
               s.hold >= HOLD_PERMANENT ? "permanent" : (s.hold == 0 ? "back to the schedule" : "timed"));
      if (s.hold > 0 && s.hold < HOLD_PERMANENT)
        ESP_LOGI(TAG, "Zone %d: hold for %u minutes", zone, s.hold);
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
// setpoint write on a scheduled zone makes a permanent hold by itself.
void ZonePauseClimate::issue_send_(const uint8_t real[2]) {
  const uint8_t zone = this->source_->get_zone();
  uint8_t dummy[2];
  bool permanent = false, timed = false;
  uint16_t real_hold = 0;
  uint8_t real_fan = NO_FAN;
  this->read_real_(dummy, permanent, &real_hold, &timed, &real_fan);
  const HoldKind view = this->hold_view_(permanent, timed, real_hold);

  Stage stages[3];
  uint8_t n = 0;
  auto hold_stage = [&](uint16_t minutes) {
    stages[n++] = Stage{STAGE_HOLD, minutes, 0, 0, 0};
  };
  auto setpoint_stage = [&](uint8_t heat, uint8_t cool) {
    stages[n++] = Stage{STAGE_SETPOINTS, 0, heat, cool, 0};
  };

  if (this->data_.goal == GOAL_PAUSE) {
    setpoint_stage(this->data_.wide_heat, this->data_.wide_cool);
    if (view != HOLD_KIND_PERMANENT)
      hold_stage(HOLD_PERMANENT);
  } else if (this->data_.goal == GOAL_RESTORE) {
    // A side that is no longer owed goes out as its real value.
    const uint8_t heat = this->data_.owed_heat ? this->data_.target_heat : real[HEAT];
    const uint8_t cool = this->data_.owed_cool ? this->data_.target_cool : real[COOL];
    const bool want_hold = this->data_.restore_hold && this->data_.owed_hold;
    HoldKind kind = want_hold ? this->restore_hold_kind_() : HOLD_KIND_NONE;
    // A timed hold goes back to the end time it had. What is left of it is worked out from
    // the thermostat's clock now, at the moment this write is built.
    const bool timed_snapshot = this->data_.hold_minutes != 0 && this->data_.hold_minutes < HOLD_PERMANENT;
    uint16_t timed_minutes = 0;
    if (want_hold && timed_snapshot) {
      const uint16_t end = this->data_.hold_end_mow;
      uint8_t weekday = 0;
      uint16_t now_min = 0;
      const bool have_clock = end != HOLD_END_UNKNOWN && this->bus_clock_(weekday, now_min);
      const uint16_t left =
          have_clock ? (uint16_t) (((uint32_t) end + MINUTES_PER_WEEK - (weekday * 1440 + now_min)) % MINUTES_PER_WEEK)
                     : 0;
      if (have_clock && left >= MIN_TIMED_HOLD && left <= MAX_TIMED_HOLD) {
        timed_minutes = left;
        kind = HOLD_KIND_TIMED;
      } else {
        // The end has gone by, or it is not known (the thermostat's clock could not be read
        // when the snapshot was taken, or cannot be read now): the zone goes back to its
        // schedule instead, exactly as a snapshot with no hold does.
        this->data_.hold_minutes = 0;
        this->data_.hold_end_mow = HOLD_END_UNKNOWN;
        kind = HOLD_KIND_NONE;
        if (!this->hold_fallback_logged_) {
          this->hold_fallback_logged_ = true;
          if (have_clock)
            ESP_LOGI(TAG,
                     "Zone %d: the hold you had until %02u:%02u has passed or ends within 15 minutes; back to the "
                     "schedule",
                     zone, (unsigned) (end % 1440 / 60), (unsigned) (end % 60));
          else
            ESP_LOGI(TAG, "Zone %d: the time your hold ran to is not known, so it cannot be put back; back to the "
                          "schedule",
                     zone);
        }
      }
    }
    switch (kind) {
      case HOLD_KIND_PERMANENT:
        setpoint_stage(heat, cool);
        if (view != HOLD_KIND_PERMANENT)
          hold_stage(HOLD_PERMANENT);
        break;
      case HOLD_KIND_TIMED:
        hold_stage(timed_minutes);
        setpoint_stage(heat, cool);
        break;
      default:
        if (!want_hold) {
          setpoint_stage(heat, cool);
        } else if (this->data_.target_changed) {
          // Scheduled zone, target edited: release first, then the values (the thermostat
          // then makes its own hold at them, like any setpoint change).
          if (view != HOLD_KIND_NONE)
            hold_stage(0);
          setpoint_stage(heat, cool);
        } else {
          // Scheduled zone: values back, then unhold (the thermostat then loads its schedule
          // values, or keeps these; either way the zone is not left wide).
          setpoint_stage(heat, cool);
          if (view != HOLD_KIND_NONE)
            hold_stage(0);
        }
        break;
    }
  } else {  // GOAL_SET
    // A pause restore that was releasing the hold was superseded by this action: the release
    // is still due, so this goal always writes its own hold kind, whatever the zone shows.
    const bool release_due = this->data_.release_due;
    switch (this->data_.set_hold) {
      case SET_HOLD_CANCEL:
        if (view != HOLD_KIND_NONE || release_due)
          hold_stage(0);
        break;
      case SET_HOLD_PERMANENT:
        if (view != HOLD_KIND_PERMANENT || release_due)
          hold_stage(HOLD_PERMANENT);
        break;
      case SET_HOLD_TIMED_NEXT:
      case SET_HOLD_TIMED_MIN: {
        // On a resend, a permanent hold with our timed hold still owed means the timed hold
        // never landed and our setpoint write made the thermostat hold permanently instead:
        // write the timed hold again rather than accept it.
        const bool timed_lost = this->resend_count_ > 0 && this->data_.owed_hold && view == HOLD_KIND_PERMANENT;
        if (view == HOLD_KIND_NONE || release_due || timed_lost) {
          const uint16_t next = this->minutes_to_next_activity_();
          uint16_t minutes = next;
          if (this->data_.set_hold == SET_HOLD_TIMED_MIN && minutes < this->minimum_hold_)
            minutes = this->minimum_hold_;
          if (minutes == 0)
            minutes = this->minimum_hold_ != 0 ? this->minimum_hold_ : 60;
          if (next == 0 && !this->hold_fallback_logged_) {
            this->hold_fallback_logged_ = true;
            ESP_LOGW(TAG, "Zone %d: schedule not readable; holding %u min instead of until the next activity", zone,
                     minutes);
          }
          this->data_.set_hold_minutes = minutes;
          this->data_.owed_hold = true;
          hold_stage(minutes);
        } else {
          this->data_.owed_hold = false;  // a timed hold keeps its timer, a permanent one stays
        }
        break;
      }
      default:  // SET_HOLD_KEEP
        if (release_due)
          hold_stage(0);
        break;
    }
    if (n > 0)
      this->data_.release_due = false;  // the hold stage below carries the release out
    if (this->data_.owed_heat || this->data_.owed_cool) {
      const uint8_t heat = this->data_.owed_heat ? this->data_.target_heat : real[HEAT];
      const uint8_t cool = this->data_.owed_cool ? this->data_.target_cool : real[COOL];
      setpoint_stage(heat, cool);
    }
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
  this->issue_stage_(stages[0]);
  for (uint8_t i = 1; i < n; i++)
    this->queue_stage_(stages[i]);
  if (n > 1)
    ESP_LOGI(TAG, "Zone %d: %d more write%s to follow, 10 s apart", zone, n - 1, n > 2 ? "s" : "");
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
    else if (real[s] == wide && wide != target)  // still at its wide value: put it back
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
    } else if (this->is_sent_(side, value) || (this->data_.goal == GOAL_RESTORE && value == wide)) {
      // A send of ours showing up late. The next send corrects it.
      this->baseline_[s] = value;
    } else if (this->data_.goal == GOAL_PAUSE && !watched[s]) {
      // The current mode does not use this side: keep the new value as its target.
      this->baseline_[s] = value;
      this->adopt_target_(side, value);
      changed = true;
      ESP_LOGW(TAG, "Zone %d is paused: %s was changed to %d by something else; kept as the new %s target", zone,
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
        ESP_LOGI(TAG, "Zone %d: %s was changed to %d by something else; that value stands", zone, SIDE_NAMES[s],
                 real[s]);
      }
      this->adopt_target_(side, real[s]);
      if (this->desired_(side) != 0)
        this->set_desired_(side, 0);
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

  // A wanted value that did not land in a mode that takes it: send again a few times, then
  // give up, let the thermostat's own values stand and show them on the card.
  if (quiet_period_just_ended && this->data_.goal != GOAL_NONE &&
      !this->satisfied_(real, permanent, timed, hold_minutes, fan)) {
    if (lands[HEAT] || lands[COOL]) {
      if (this->resend_count_ < MAX_RESENDS) {
        this->resend_count_++;
        this->send_due_ = true;
        ESP_LOGW(TAG, "Zone %d: the thermostat has not taken everything yet (it shows %d / %d); sending again (%d of %d)",
                 zone, real[HEAT], real[COOL], this->resend_count_, MAX_RESENDS);
      } else if (this->data_.goal == GOAL_PAUSE) {
        // A pause is not ended here: the zone is still paused and its snapshot must survive.
        // Log once and stop resending; the pause switch is the way out.
        if (this->resend_count_ == MAX_RESENDS) {
          this->resend_count_++;  // log once
          ESP_LOGE(TAG, "Zone %d: the thermostat still shows %d / %d and has not taken the pause. "
                        "Check the Actual setpoint sensors and Hold State.",
                   zone, real[HEAT], real[COOL]);
        }
      } else {
        ESP_LOGE(TAG, "Zone %d: gave up: the thermostat did not take the change; it holds %d / %d, hold %s", zone,
                 real[HEAT], real[COOL], permanent ? "permanent" : (timed ? "timed" : "none"));
        this->adopt_target_(HEAT, real[HEAT]);
        this->adopt_target_(COOL, real[COOL]);
        this->set_goal_(GOAL_NONE);  // clears every owed part, the preset and the due release
        changed = true;
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
      this->set_owed_(side, false);
      this->adopt_target_(side, real[s]);
    }
  }
  if (this->data_.goal == GOAL_PAUSE && (deviated[HEAT] || deviated[COOL])) {
    this->end_pause_by_deviation_(real, deviated);
    return;
  }
  if (this->data_.goal != GOAL_NONE) {
    ESP_LOGI(TAG, "Zone %d: %s after the restart", zone,
             this->data_.goal == GOAL_PAUSE ? "still paused" : "finishing what was being applied");
    this->send_due_ = true;
  }
  this->save_();
  this->publish_all_();
}

// A desired value whose side the confirmed mode now takes becomes an edit (hold rule of an
// edit, at least the minimum hold on a scheduled zone).
void ZonePauseClimate::deliver_desired_(const uint8_t real[2]) {
  bool lands[2];
  landing_sides_(this->confirmed_mode_, lands[HEAT], lands[COOL]);
  bool any = false;
  bool dropped = false;
  for (uint8_t s = 0; s < 2; s++) {
    const Side side = static_cast<Side>(s);
    const uint8_t value = this->desired_(side);
    if (value == 0 || !lands[s])
      continue;
    if (value == real[s]) {
      // The thermostat is already there: nothing to send, and no goal for it.
      this->set_desired_(side, 0);
      dropped = true;
      ESP_LOGI(TAG, "Zone %d: the mode now takes %s and it is already at %d; nothing to send",
               this->source_->get_zone(), SIDE_NAMES[s], value);
      continue;
    }
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
    if (this->data_.set_hold == SET_HOLD_KEEP)
      this->data_.set_hold = SET_HOLD_TIMED_MIN;
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

  // A desired value the mode never took is dropped after a day. Nothing else drops it here;
  // a change to that side made for another reason drops it below.
  if (this->data_.desired_heat != 0 || this->data_.desired_cool != 0) {
    bool expired = false;
    for (uint8_t s = 0; s < 2; s++) {
      const Side side = static_cast<Side>(s);
      const uint8_t want = this->desired_(side);
      const uint16_t age = side == HEAT ? this->data_.desired_heat_age : this->data_.desired_cool_age;
      if (want == 0 || age < DESIRED_MAX_AGE_MIN)
        continue;
      ESP_LOGI(TAG, "Zone %d: the %s %d you set a day ago was never used and is dropped", this->source_->get_zone(),
               SIDE_NAMES[s], want);
      this->set_desired_(side, 0);  // clears the value and its age
      expired = true;
    }
    if (expired) {
      this->save_();
      this->publish_all_();
    }
  }

  // Nothing of ours in flight and a side moved: whoever did it wins, so that side's desired
  // value is dropped.
  if (this->data_.goal == GOAL_NONE && this->last_real_valid_) {
    bool dropped = false;
    for (uint8_t s = 0; s < 2; s++) {
      const Side side = static_cast<Side>(s);
      const uint8_t want = this->desired_(side);
      if (want == 0 || real[s] == this->last_real_[s] || real[s] == want)
        continue;
      ESP_LOGI(TAG, "Zone %d: %s changed to %d by something else; the %s %d you set earlier is dropped",
               this->source_->get_zone(), SIDE_NAMES[s], real[s], SIDE_NAMES[s], want);
      this->set_desired_(side, 0);
      dropped = true;
    }
    if (dropped) {
      this->save_();
      this->publish_all_();
    }
  }
  this->last_real_[HEAT] = real[HEAT];
  this->last_real_[COOL] = real[COOL];
  this->last_real_valid_ = true;

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
        this->in_quiet_ = false;
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

bool ZonePauseClimate::schedule_ok_() const {
  if (!this->schedule_read_valid_ || millis() - this->schedule_read_ms_ > SCHEDULE_STALE_MS)
    return false;
  const auto *row = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT,
                                                (uint16_t) (infinitesp::REG_TSTAT_SCHEDULE + this->source_->get_zone() - 1));
  return row != nullptr && row->size() >= 70;
}

// Verified on this thermostat (2026-09-22): 7 days from Sunday x 5 periods x (start minute
// / 15, activity); a disabled period has start 0x60 (24:00). Bus clock: weekday 0 = Sunday.
uint8_t ZonePauseClimate::schedule_activity_now_() const {
  uint8_t weekday;
  uint16_t minutes;
  if (!this->schedule_ok_() || !this->bus_clock_(weekday, minutes))
    return NO_PRESET;
  const auto *row = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT,
                                                (uint16_t) (infinitesp::REG_TSTAT_SCHEDULE + this->source_->get_zone() - 1));
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
  if (!this->schedule_ok_() || !this->bus_clock_(weekday, minutes))
    return 0;
  const auto *row = this->parent_->get_register(infinitesp::ADDR_THERMOSTAT,
                                                (uint16_t) (infinitesp::REG_TSTAT_SCHEDULE + this->source_->get_zone() - 1));
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
      return delta > MAX_TIMED_HOLD ? MAX_TIMED_HOLD : (uint16_t) delta;
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
    // The activity the person chose, while it is being applied.
    switch (this->data_.set_preset) {
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
  const bool govern[2] = {lands[HEAT] || (!lands[HEAT] && !lands[COOL]), lands[COOL] || (!lands[HEAT] && !lands[COOL])};
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
  if (matched != NO_PRESET) {
    switch (matched) {
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
    return;
  }
  if (permanent)
    this->set_custom_preset_(infinitesp::PRESET_HOLD_PERM);
  else if (timed)
    this->set_custom_preset_(infinitesp::PRESET_HOLD_TIMED);
  else
    this->set_custom_preset_(infinitesp::PRESET_SCHEDULE);
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
