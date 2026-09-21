# infinitesp-zone-pause

A per-zone **Pause** switch for Carrier Infinity / Bryant Evolution systems controlled
through [InfinitESP](https://github.com/nebulous/infinitesp).

This is a small ESPHome external component that sits in front of an InfinitESP zone.
It does not replace or fork InfinitESP, and it never writes to the bus itself. It only
calls the InfinitESP hub's public methods.

> **Status: experimental.** Compiles against ESPHome 2026.9 and InfinitESP commit
> `4332fb8`. Testing on a live system is in progress. Read the [NOTICE](NOTICE).

## What it does

Pausing a zone stops it calling for heating or cooling by holding its setpoints wide
(default heat 50 °F / cool 85 °F) on a permanent hold. Turning pause off puts the zone
back the way it was. Typical use: a door or window is open, so a home automation hub
pauses the zones that would be heating or cooling the outdoors.

The component knows nothing about doors, windows or reasons. It offers one switch per
zone. Your automations decide when to use it.

Per zone you get:

| Entity | What it is |
|---|---|
| Thermostat (climate) | Always shows the **target** settings, never the wide pause values |
| Pause switch | On = paused, off = running. It is the only paused indicator |
| Actual heat / cool setpoint sensors (optional) | What the thermostat really holds right now |

Works with Home Assistant's standard thermostat card and a standard toggle. No custom
card is needed.

## Behavior

1. **Pause on:** remember the zone's setpoints and hold state, then hold the zone at the
   wide values on a permanent hold.
2. **Pause off:** put the remembered state back. A zone that was following its schedule
   returns to the schedule as it stands now. A permanent hold comes back. A timed hold
   comes back with the time it had left. If it had 15 minutes or less left, or the
   ESP32 restarted meanwhile, the zone returns to its schedule instead.
3. **Mode and fan are not affected by pause.** They pass straight through, paused or not.
4. **A target change while paused is remembered, not applied.** It is what the zone
   resumes to.
5. **A change at the wall control wins.** If a paused zone's setpoints or hold are changed
   by anything else (wall control, the Carrier app, vacation mode), the pause ends, the
   switch turns off and that change stands.
6. **A pause lasts until it is ended.** There are no timers.
7. **Pause state survives a restart** of the ESP32 and is checked against the thermostat
   afterwards.
8. **Nothing is assumed.** A pause or resume counts only once the thermostat's own reply
   confirms it (checked 30 s after the command, one resend). If the thermostat does not
   accept a pause, the switch turns back off. Half a pause (wide setpoints without the
   permanent hold) is undone.

## Configuration

Requires InfinitESP in active (SAM) mode. Give the InfinitESP zone an `id`, mark it
`internal: true`, and let the wrapper take the zone's name:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nebulous/infinitesp
      ref: <commit>
  - source:
      type: git
      url: https://github.com/Electrified-Home/infinitesp-zone-pause
      ref: <commit>
    components: [zone_pause]

climate:
  - platform: infinitesp
    infinitesp_id: infinitesp_hub
    id: upstairs_raw
    name: "Upstairs Raw"
    internal: true
    zone: 1

  - platform: zone_pause
    infinitesp_id: infinitesp_hub
    source_id: upstairs_raw
    name: "Upstairs"
    pause_heat_setpoint: 50   # optional, whole degrees F, default 50
    pause_cool_setpoint: 85   # optional, whole degrees F, default 85
    pause_switch:
      name: "Upstairs Pause"
    actual_heat_setpoint:     # optional
      name: "Upstairs Actual Heat Setpoint"
    actual_cool_setpoint:     # optional
      name: "Upstairs Actual Cool Setpoint"
```

Pin both components to a commit. This component reads InfinitESP's register layout and
hub methods directly, so an InfinitESP update may need a matching update here.

## Known limits

- The wide values are configured in whole degrees Fahrenheit and must be at least
  2 degrees apart. They must be values the thermostat accepts; if it refuses them the
  pause ends and the log says so.
- Presets are ignored while a zone is paused.
- Vacation mode starting ends a pause.
- A paused zone still heats below the wide heat value and cools above the wide cool
  value. That is intended: it is the freeze guard.

## Design notes

- Not an ESPHome `Component`. It registers with the InfinitESP hub as one of its
  lightweight entities and is driven by the hub's register-update notifications, the
  same pattern InfinitESP's own entities use.
- Truth is read from the thermostat's own register reply, not from the hub's optimistic
  copy that is updated when a command is queued.

## License

MIT. See [LICENSE](LICENSE) and [NOTICE](NOTICE). InfinitESP is the work of
[nebulous](https://github.com/nebulous) and is also MIT licensed.
