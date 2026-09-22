#pragma once

#include "esphome/core/preferences.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/infinitesp/infinitesp.h"
#include "esphome/components/infinitesp/infinitesp_climate.h"

namespace esphome {
namespace zone_pause {

class ZonePauseClimate;

class ZonePauseSwitch : public switch_::Switch {
 public:
  void set_parent(ZonePauseClimate *parent) { parent_ = parent; }

 protected:
  void write_state(bool state) override;
  ZonePauseClimate *parent_{nullptr};
};

// Proxy thermostat shown to Home Assistant for one Carrier zone. It always carries the
// TARGET settings. Behavior: see the README, sections "Behavior" and "Presets and holds".
// It knows nothing about why a zone is paused; it only obeys the pause switch.
//
// Deliberately NOT an ESPHome Component. InfinitESP documents an outage caused by
// extra Component registrations next to its hub, so this follows the same
// lightweight pattern as InfinitESP's own entities: it registers with the hub as
// an InfinitESPEntity and is driven by the hub's register-update notifications,
// which arrive many times a second while the bus is alive.
class ZonePauseClimate : public climate::Climate, public infinitesp::InfinitESPEntity {
 public:
  // Called once from generated code, after the entity is registered.
  void init();

  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;
  void on_register_update(uint8_t device_addr, uint16_t register_key) override;

  void set_source(infinitesp::InfinitESPClimate *source) { source_ = source; }
  void set_pause_setpoints(uint8_t heat_f, uint8_t cool_f) {
    pause_heat_f_ = heat_f;
    pause_cool_f_ = cool_f;
  }
  void set_minimum_hold(uint16_t minutes) { minimum_hold_ = minutes; }
  void set_pause_switch(ZonePauseSwitch *sw) { pause_switch_ = sw; }
  void set_actual_heat_sensor(sensor::Sensor *s) { actual_heat_sensor_ = s; }
  void set_actual_cool_sensor(sensor::Sensor *s) { actual_cool_sensor_ = s; }

  // send_now false: the caller is about to replace this goal in the same card action, so the
  // send is left for it (one write sequence, not two).
  void request_pause(bool pause, bool send_now = true);
  bool is_paused() const { return data_.paused; }

 protected:
  enum Side : uint8_t { HEAT = 0, COOL = 1 };
  // What the component is bringing the thermostat to.
  enum Goal : uint8_t { GOAL_NONE = 0, GOAL_PAUSE = 1, GOAL_RESTORE = 2, GOAL_SET = 3 };
  enum HoldKind : uint8_t { HOLD_KIND_NONE, HOLD_KIND_PERMANENT, HOLD_KIND_TIMED };
  // The hold a SET goal wants (decided against the hold state when the write goes out).
  enum SetHold : uint8_t {
    SET_HOLD_KEEP = 0,        // no hold write
    SET_HOLD_TIMED_NEXT = 1,  // until the next scheduled activity, exactly (presets)
    SET_HOLD_TIMED_MIN = 2,   // until the next activity, at least minimum_hold (edits)
    SET_HOLD_PERMANENT = 3,
    SET_HOLD_CANCEL = 4,
  };
  enum StageKind : uint8_t { STAGE_HOLD, STAGE_SETPOINTS, STAGE_FAN };
  struct Stage {
    StageKind kind;
    uint16_t hold;
    uint8_t heat;
    uint8_t cool;
    uint8_t fan;
  };
  static const uint8_t NO_FAN = 0xFF;
  static const uint8_t NO_PRESET = 0xFF;

  // Saved to flash so a pause, a restore or a card action that has not landed yet survives
  // a restart. Nothing about the send mechanics (stages, timers) is kept here;
  // `set_hold_minutes` is the value being judged.
  struct Saved {
    uint8_t version;
    bool paused;
    uint8_t goal;
    uint8_t target_heat;    // bus units. PAUSE: the snapshot. RESTORE / SET: the value being put
                            // back or set (an edit or an accepted outside change replaces it).
    uint8_t target_cool;
    uint16_t hold_minutes;  // hold when paused: 0 none, 0xFFFF permanent, else timed
    bool target_changed;    // the target was edited while paused
    uint8_t wide_heat;      // wide values in force for this pause
    uint8_t wide_cool;
    bool owed_heat;         // RESTORE / SET: parts that have not landed yet
    bool owed_cool;
    bool owed_hold;
    bool restore_hold;      // RESTORE: whether the hold is part of it at all
    // SET goals (card edits, presets, hold commands)
    uint8_t set_fan;        // NO_FAN or a fan code
    bool owed_fan;
    uint8_t set_hold;       // SetHold
    uint16_t set_hold_minutes;  // the timed hold actually written, for judging
    uint8_t set_preset;     // activity index the card chose, NO_PRESET for a plain edit
    // Values the current mode could not take, kept for when it can (0 = none)
    uint8_t desired_heat;
    uint8_t desired_cool;
    uint16_t desired_heat_age;  // minutes
    uint16_t desired_cool_age;
    // A pause restore was releasing the hold when a card action superseded it: the release
    // is still due, so the next SET goal writes its hold kind instead of keeping what the
    // thermostat shows.
    bool release_due;
    // A timed snapshot hold is remembered by the END it had, as minutes of the week on the
    // thermostat's clock (Sunday 00:00 = 0), so the restore puts back that same end time
    // however long the pause lasted and whether or not the board restarted.
    // 0xFFFF: not known (the clock could not be read when the snapshot was taken).
    uint16_t hold_end_mow;
  } __attribute__((packed));

  void ensure_started_();
  bool read_real_(uint8_t real[2], bool &permanent, uint16_t *hold_minutes = nullptr, bool *timed = nullptr,
                  uint8_t *fan = nullptr) const;
  bool real_is_fresh_() const;
  uint8_t read_confirmed_mode_() const;
  static void landing_sides_(uint8_t mode, bool &heat, bool &cool);
  static void watched_sides_(uint8_t mode, bool &heat, bool &cool);
  static const char *mode_name_(uint8_t mode);
  bool bus_ready_() const;
  uint8_t pause_heat_bus_() const;
  uint8_t pause_cool_bus_() const;
  uint8_t goal_value_(Side side) const;
  bool owed_(Side side) const { return side == HEAT ? data_.owed_heat : data_.owed_cool; }
  void set_owed_(Side side, bool owed) {
    if (side == HEAT)
      data_.owed_heat = owed;
    else
      data_.owed_cool = owed;
  }
  uint8_t desired_(Side side) const { return side == HEAT ? data_.desired_heat : data_.desired_cool; }
  void set_desired_(Side side, uint8_t value);
  HoldKind restore_hold_kind_() const;
  HoldKind hold_view_(bool permanent, bool timed, uint16_t hold_minutes) const;
  void set_goal_(Goal goal);
  void begin_set_goal_();
  void add_sent_(Side side, uint8_t value);
  bool is_sent_(Side side, uint8_t value) const;
  bool satisfied_(const uint8_t real[2], bool permanent, bool timed, uint16_t hold_minutes, uint8_t fan) const;
  bool hold_landed_(bool permanent, bool timed, uint16_t hold_minutes) const;
  void adopt_target_(Side side, uint8_t value);
  void queue_stage_(const Stage &s);
  void issue_stage_(const Stage &s);
  void issue_send_(const uint8_t real[2]);
  void judge_(const uint8_t real[2], bool permanent, bool timed, uint16_t hold_minutes, uint8_t fan,
              bool quiet_period_just_ended);
  void end_pause_by_deviation_(const uint8_t real[2], const bool deviated[2]);
  void reconcile_after_restart_(const uint8_t real[2]);
  void deliver_desired_(const uint8_t real[2]);
  void evaluate_();
  void mirror_from_source_();
  void infer_preset_();
  void publish_actual_(uint8_t heat, uint8_t cool);
  void publish_all_();
  void save_();
  void flush_();

  // Schedule (register 0x4002 + zone-1) and bus clock
  bool bus_clock_(uint8_t &weekday, uint16_t &minutes) const;
  bool schedule_ok_() const;
  uint16_t minutes_to_next_activity_() const;
  uint8_t schedule_activity_now_() const;
  bool comfort_setpoints_(uint8_t activity, uint8_t &heat, uint8_t &cool, uint8_t &fan) const;

  infinitesp::InfinitESPClimate *source_{nullptr};
  ZonePauseSwitch *pause_switch_{nullptr};
  sensor::Sensor *actual_heat_sensor_{nullptr};
  sensor::Sensor *actual_cool_sensor_{nullptr};

  uint8_t pause_heat_f_{50};
  uint8_t pause_cool_f_{85};
  uint16_t minimum_hold_{60};

  Saved data_{};
  ESPPreferenceObject pref_;
  bool started_{false};
  bool needs_reconcile_{false};  // state came from flash; check it against reality first

  // Sending: one bus write at a time (gate shared by every zone), then a quiet period in
  // which nothing is judged.
  bool send_due_{false};
  bool in_quiet_{false};
  uint32_t last_send_ms_{0};      // this zone's last write
  uint8_t confirmed_mode_{0xFF};  // the thermostat's own mode nibble, 0xFF until known
  uint8_t resend_count_{0};       // sends repeated because a wanted value did not land
  Stage pending_[2]{};            // later stages of the current send, one gap apart
  uint8_t pending_count_{0};
  bool have_zones_reply_{false};  // a real 3B03 reply from the thermostat has been seen
  uint32_t last_zones_reply_ms_{0};
  // The hold this component last wrote and when; within 30 s it defines the hold state.
  HoldKind own_hold_kind_{HOLD_KIND_NONE};
  uint32_t own_hold_ms_{0};
  bool own_hold_valid_{false};
  // A system mode change waiting for the shared gate (a mode write is two bus frames).
  climate::ClimateMode pending_mode_{climate::CLIMATE_MODE_OFF};
  bool pending_mode_valid_{false};
  // Logged once per goal: the schedule fallback ("holding N min instead of until the next
  // activity"), or a restore whose timed hold had already ended.
  bool hold_fallback_logged_{false};

  // Judging: per side, the last real value accounted for, whether the goal value was seen
  // during the quiet period, and the values this component sent since the goal was NONE.
  uint8_t baseline_[2]{0, 0};
  bool goal_seen_[2]{false, false};
  uint8_t sent_[2][4]{};
  uint32_t sent_ms_[2][4]{};
  uint8_t sent_count_[2]{0, 0};

  uint32_t minute_accum_ms_{0};
  uint32_t last_tick_ms_{0};
  uint8_t last_actual_heat_{0};
  uint8_t last_actual_cool_{0};
  bool switch_published_{false};
  bool dirty_{false};  // saved state changed but not yet written to flash
  uint32_t last_sync_ms_{0};

  // Schedule row: read once at boot and every 6 hours (each read costs the hub one missed poll).
  uint32_t schedule_poll_at_ms_{0};
  uint32_t schedule_read_ms_{0};
  bool schedule_read_valid_{false};
  // The last real setpoints seen, to spot a change made by something else while nothing of
  // ours is in flight.
  uint8_t last_real_[2]{0, 0};
  bool last_real_valid_{false};
};

}  // namespace zone_pause
}  // namespace esphome
