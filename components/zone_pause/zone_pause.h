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
// TARGET settings. Behavior: see the README, section "Behavior".
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
  void set_pause_switch(ZonePauseSwitch *sw) { pause_switch_ = sw; }
  void set_actual_heat_sensor(sensor::Sensor *s) { actual_heat_sensor_ = s; }
  void set_actual_cool_sensor(sensor::Sensor *s) { actual_cool_sensor_ = s; }

  void request_pause(bool pause);
  bool is_paused() const { return data_.paused; }

 protected:
  enum Side : uint8_t { HEAT = 0, COOL = 1 };
  // What the component is bringing the thermostat to.
  enum Goal : uint8_t { GOAL_NONE = 0, GOAL_PAUSE = 1, GOAL_RESTORE = 2 };
  enum HoldKind : uint8_t { HOLD_KIND_NONE, HOLD_KIND_PERMANENT, HOLD_KIND_TIMED };
  enum Stage2 : uint8_t { STAGE_NONE, STAGE_HOLD, STAGE_SETPOINTS };

  // Saved to flash so a pause, or a restore that has not landed yet, survives a restart.
  // Nothing per-send is kept here.
  struct Saved {
    uint8_t version;
    bool paused;
    uint8_t goal;
    uint8_t target_heat;    // bus units. Goal PAUSE: the snapshot. Goal RESTORE: the value being put
                            // back (an edit or an accepted outside change replaces it). The card shows it.
    uint8_t target_cool;
    uint16_t hold_minutes;  // hold when paused: 0 none, 0xFFFF permanent, else timed
    bool target_changed;    // the target was edited while paused
    uint8_t wide_heat;   // wide values in force for this pause
    uint8_t wide_cool;
    bool owed_heat;         // goal RESTORE: parts that have not landed yet
    bool owed_cool;
    bool owed_hold;
    bool restore_hold;      // goal RESTORE: whether the hold is part of it at all
  } __attribute__((packed));

  void ensure_started_();
  bool read_real_(uint8_t real[2], bool &permanent, uint16_t *hold_minutes = nullptr) const;
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
  HoldKind restore_hold_kind_() const;
  void set_goal_(Goal goal);
  void add_sent_(Side side, uint8_t value);
  bool is_sent_(Side side, uint8_t value) const;
  bool satisfied_(const uint8_t real[2], bool permanent) const;
  void adopt_target_(Side side, uint8_t value);
  void issue_send_(const uint8_t real[2]);
  void issue_stage2_();
  void judge_(const uint8_t real[2], bool permanent, bool quiet_period_just_ended);
  void end_pause_by_deviation_(const uint8_t real[2], const bool deviated[2]);
  void reconcile_after_restart_(const uint8_t real[2]);
  void evaluate_();
  void mirror_from_source_();
  void publish_actual_(uint8_t heat, uint8_t cool);
  void publish_all_();
  void save_();
  void flush_();

  infinitesp::InfinitESPClimate *source_{nullptr};
  ZonePauseSwitch *pause_switch_{nullptr};
  sensor::Sensor *actual_heat_sensor_{nullptr};
  sensor::Sensor *actual_cool_sensor_{nullptr};

  uint8_t pause_heat_f_{50};
  uint8_t pause_cool_f_{85};

  Saved data_{};
  ESPPreferenceObject pref_;
  bool started_{false};
  bool needs_reconcile_{false};  // state came from flash; check it against reality first

  // Sending: one send at a time, then a quiet period in which nothing is judged.
  bool send_due_{false};
  bool ever_sent_{false};
  bool in_quiet_{false};
  uint32_t last_send_ms_{0};
  uint8_t confirmed_mode_{0xFF};  // the thermostat's own mode nibble, 0xFF until known
  uint8_t resend_count_{0};       // sends repeated because a wanted value did not land
  // Second half of a send, issued one gap after the first (one bus write at a time).
  Stage2 stage2_{STAGE_NONE};
  uint16_t stage2_hold_{0};
  uint8_t stage2_heat_{0};
  uint8_t stage2_cool_{0};
  bool have_zones_reply_{false};  // a real 3B03 reply from the thermostat has been seen
  uint32_t last_zones_reply_ms_{0};

  // Judging: per side, the last real value accounted for, whether the goal value was seen
  // during the quiet period, and the values this component sent since the goal was NONE.
  uint8_t baseline_[2]{0, 0};
  bool goal_seen_[2]{false, false};
  uint8_t sent_[2][4]{};
  uint32_t sent_ms_[2][4]{};
  uint8_t sent_count_[2]{0, 0};

  // The minutes paused are not saved, so after a restart a timed hold cannot be put back.
  bool restarted_since_pause_{false};
  uint16_t paused_minutes_{0};  // minutes since the snapshot was taken
  uint32_t minute_accum_ms_{0};
  uint32_t last_tick_ms_{0};
  uint8_t last_actual_heat_{0};
  uint8_t last_actual_cool_{0};
  bool switch_published_{false};
  bool dirty_{false};  // saved state changed but not yet written to flash
  uint32_t last_sync_ms_{0};
};

}  // namespace zone_pause
}  // namespace esphome
