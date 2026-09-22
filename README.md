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
| Actual heat / cool setpoint sensors (required) | What the thermostat really holds right now. The card never shows this, so these are the truth channel |

Works with Home Assistant's standard thermostat card and a standard toggle. No custom
card is needed.

## What we learned about the thermostat (Infinity Touch, SAM path)

Observed on one system; treat it as a starting point, not a specification:

| System mode | Heat setpoint write | Cool setpoint write |
|---|---|---|
| Heat | accepted | **ignored** |
| Cool | not tested | accepted |
| Auto | not tested | not tested |
| Off | **ignored** | **ignored** |

The thermostat only takes the setpoint its current mode uses, and nothing while off. A
mode change does not alter setpoints. InfinitESP repeats each write three times without an acknowledgement, the thermostat's own
reply can lag a change by several seconds, and the thermostat drops a write when another
write follows it within a fraction of a second (only the last of a burst is processed), so
this component never sends two writes back to back. The behavior below follows from that.

## Behavior

1. **Pause is a mode of the zone.** Switch on: remember the zone's setpoints and hold (the
   snapshot), send both wide setpoints and a permanent hold. The zone is paused at once, in
   any system mode, even when the thermostat will not take the values yet.
2. **A system mode change while paused sends again**, so the side the new mode uses goes
   wide when the mode allows it (off to heat, heat to cool, and so on).
3. **Only two things end a pause.** The switch is turned off: the snapshot is put back. Or
   a setpoint the current mode uses is changed by anything else (wall control, the Carrier
   app, another integration) to something other than its pause value: the pause is over,
   that value stands. In heat mode only the heat setpoint counts, in cool mode only the cool
   setpoint, in auto and off both. A change to the other setpoint is kept as that side's
   new target and the pause carries on.
4. **A send that does not land never ends anything.** It is repeated up to three times when
   the thermostat's own reply shows the value did not arrive, and again on the next mode
   change.
5. **Putting the zone back** follows the hold it was on: a permanent hold comes back with
   the setpoints; a scheduled zone gets its values back and the hold released; a timed hold
   comes back with the time it had left if that is more than 15 minutes (the thermostat's
   minimum), otherwise the zone returns to its schedule. A part the current mode will not
   take stays owed and is delivered when the mode allows; meanwhile the card keeps showing
   the target.
6. **Mode and fan are not affected by pause.** A target edited while paused is remembered
   and applied when the pause ends. Presets are ignored while paused.
7. **Pause state survives a restart** of the ESP32, including a power cut, and is checked
   against the thermostat afterwards.
8. **The component never turns the switch on by itself**, and there are no timers: a pause
   lasts until it is ended.

Judging is done only from the thermostat's own register replies, and not for 30 seconds
after one of the component's own sends (its write is still being repeated and the reply
can lag). Nothing visible waits for that: the switch, the card and the Actual sensors
update at once.

## Configuration

Requires InfinitESP in active (SAM) mode. Give the InfinitESP zone an `id`, mark it
`internal: true`, name it `"<Zone> Climate"`, and let the wrapper take the zone's name.

**The hidden block's name matters.** InfinitESP names the zone's other entities
(Temperature, Humidity, Fan Mode, Hold Minutes, ...) after that block and drops a trailing
"Climate". With `"Upstairs Climate"` they stay `Upstairs Temperature` and so on, exactly as
before. Any other name (say `"Upstairs Raw"`) renames every one of them in Home Assistant.

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
    name: "Upstairs Climate"   # see the note above
    internal: true
    zone: 1

  - platform: zone_pause
    infinitesp_id: infinitesp_hub
    source_id: upstairs_raw    # one zone_pause block per InfinitESP zone, never two
    name: "Upstairs"
    pause_heat_setpoint: 50    # optional, whole degrees FAHRENHEIT, default 50
    pause_cool_setpoint: 85    # optional, whole degrees FAHRENHEIT, default 85
    pause_switch:
      name: "Upstairs Pause"
    actual_heat_setpoint:      # required
      name: "Upstairs Actual Heat Setpoint"
    actual_cool_setpoint:      # required
      name: "Upstairs Actual Cool Setpoint"
```

Pin both components to a commit. This component reads InfinitESP's register layout and
hub methods directly and relies on its first-in first-out write queue, so an InfinitESP
update may need a matching update here.

## Known limits

- The wide values are configured in whole degrees Fahrenheit and must be at least
  2 degrees apart. They must be values the thermostat accepts.
- When a pause cannot start (bus offline, no recent answer from the thermostat) or ends by
  itself, the switch simply reads off; the reason is in the ESPHome log.
- **Turn every pause switch off before installing a firmware update.** An update that
  changes the saved record forgets a pause while the thermostat keeps the zone wide.
- A change made at the wall in the first seconds after a pause or unpause can be overwritten
  once by the repeats of the component's own write; made again, it counts. Noticing a change
  can take up to 30 seconds right after a send, a few seconds otherwise.
- A pause ended by a change at the wall on a zone that was following its schedule leaves
  the pause's permanent hold in place (releasing it would discard the value just set).
- InfinitESP's own Hold Minutes / Hold Until entities still work on a paused zone; a timed
  hold set there ends the pause when it runs out.
- Vacation mode starting ends a pause.
- A paused zone still heats below the wide heat value and cools above the wide cool
  value. That is intended: it is the freeze guard.

## Design notes

- Not an ESPHome `Component`. It registers with the InfinitESP hub as one of its
  lightweight entities and is driven by the hub's register-update notifications, the
  same pattern InfinitESP's own entities use.
- Truth is read from the thermostat's own register replies, not from the hub's optimistic
  copy that is updated when a command is queued.
- While a zone is paused, or still being put back, setpoint edits are written by this
  component itself rather than forwarded, because the wrapped entity would send its cached
  real value (a wide one) for the other setpoint.

## License

MIT. See [LICENSE](LICENSE) and [NOTICE](NOTICE). InfinitESP is the work of
[nebulous](https://github.com/nebulous) and is also MIT licensed.
