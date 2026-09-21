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

// Proxy thermostat shown to Home Assistant for one Carrier zone. It always carries
// the TARGET settings. While the zone is paused the target is remembered but not
// applied, and the real zone is held at the wide pause setpoints. It knows nothing
// about why a zone is paused; it only obeys the pause switch.
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
  // Saved to flash so a paused zone survives a restart of the board.
  struct Saved {
    uint8_t version;
    bool paused;
    uint8_t target_heat;    // bus units; what the zone resumes to
    uint8_t target_cool;
    uint16_t hold_minutes;  // hold state when paused: 0 none, 0xFFFF permanent, else timed
    bool target_changed;    // the target was edited while paused
    uint8_t applied_heat;   // wide values the thermostat actually adopted
    uint8_t applied_cool;
  } __attribute__((packed));

  enum Phase : uint8_t { PHASE_IDLE, PHASE_PAUSING, PHASE_RESUMING };

  void ensure_started_();
  bool read_actual_(uint8_t &heat, uint8_t &cool) const;
  bool bus_ready_() const;
  uint8_t pause_heat_bus_() const;
  uint8_t pause_cool_bus_() const;
  void send_pause_writes_();
  void send_resume_writes_();
  void start_phase_(Phase phase);
  void end_pause_without_restore_(const char *reason);
  void evaluate_();
  void mirror_from_source_();
  void publish_actual_(uint8_t heat, uint8_t cool);
  void publish_all_();
  void save_();

  infinitesp::InfinitESPClimate *source_{nullptr};
  ZonePauseSwitch *pause_switch_{nullptr};
  sensor::Sensor *actual_heat_sensor_{nullptr};
  sensor::Sensor *actual_cool_sensor_{nullptr};

  uint8_t pause_heat_f_{50};
  uint8_t pause_cool_f_{85};

  Saved data_{};
  ESPPreferenceObject pref_;
  bool started_{false};

  Phase phase_{PHASE_IDLE};
  uint32_t phase_started_ms_{0};
  uint8_t phase_attempts_{0};
  uint8_t pre_heat_{0};  // the zone's setpoints just before pausing, to tell whether the write took
  uint8_t pre_cool_{0};
  bool queued_request_valid_{false};
  bool queued_request_{false};
  bool needs_reconcile_{false};  // paused state came from flash; check it against reality
  bool restarted_since_pause_{false};
  uint16_t paused_minutes_{0};
  uint32_t minute_accum_ms_{0};
  uint32_t last_tick_ms_{0};
  uint8_t last_actual_heat_{0};
  uint8_t last_actual_cool_{0};
  bool switch_published_{false};
  bool hold_lost_{false};  // paused, wide setpoints still in place, but the hold is no longer permanent
  uint32_t hold_lost_since_ms_{0};
};

}  // namespace zone_pause
}  // namespace esphome
