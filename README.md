# infinitesp-zone-pause

A per-zone **Pause** switch for Carrier Infinity / Bryant Evolution systems controlled
through [InfinitESP](https://github.com/nebulous/infinitesp).

This is a small ESPHome external component that sits in front of an InfinitESP zone.
It does not replace or fork InfinitESP, and it never writes to the bus itself. It only
calls the InfinitESP hub's public methods.

> **Status: experimental.** Compiles against ESPHome 2026.9 and InfinitESP commit
> `4332fb8`. The pause feature has passed its live tests; the presets and holds feature is
> built and reviewed, with its live tests in progress. Read the [NOTICE](NOTICE).

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

The same component also makes the card's presets (Wake, Home, Away, Sleep, Hold
Indefinitely, Per Schedule) and temperature edits behave like the wall control. See
[Presets and holds](#presets-and-holds).

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

Also observed, and relied on by the presets and holds feature:

- The thermostat enforces its 2 degree heat/cool gap itself, moving the side that was not
  just edited.
- A plain setpoint write on a zone following its schedule makes the thermostat create a
  permanent hold of its own.
- A timed-hold write resets that zone's setpoints to the schedule's current values, so the
  setpoints must be written after it, not before. A setpoint write after a timed hold keeps
  the timer. A permanent-hold write keeps the setpoints. A hold write of 0 minutes is a real
  cancel.
- A temperature bump at the wall on a scheduled zone arms a timed hold until the next
  scheduled activity (the wall asks to confirm that default). A bump on a timed hold keeps
  the timer; on a permanent hold it stays permanent.
- Picking an activity at the wall changes only setpoints and fan. No byte on the bus carries
  the activity's name.
- Each zone's weekly schedule can be read from the thermostat: seven days of five periods,
  each a start time (in 15 minute steps; a disabled period reads 24:00) and one of four
  activities. The thermostat also serves its clock as day of week and minutes since
  midnight.
- InfinitESP's Hold Minutes reads 0 both for no hold and for a permanent hold; its Hold
  State tells them apart.

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
4. **A send that does not land is repeated once** when the thermostat's own reply shows
   the value did not arrive (the hub itself already repeats every write three times). After that the component gives up, says so in the
   log as an error, and the card shows what the thermostat really holds. A pause is the one
   exception: it stays paused and simply stops resending.
5. **Putting the zone back** follows the hold it was on: a permanent hold comes back with
   the setpoints; a scheduled zone gets its values back and the hold released; a timed hold
   comes back to the same end time it had (the snapshot keeps the end time from the
   thermostat's own clock, so a restart or a long pause does not change it), written before the
   setpoints because a timed-hold write resets a zone to its schedule values. If that end time
   has already passed, the zone returns to its schedule. A part the current mode will not
   take stays owed and is delivered when the mode allows; meanwhile the card keeps showing
   the target.
6. **Mode and fan are not affected by pause.** A target edited while paused is remembered
   and applied when the pause ends. Presets picked while paused resume the zone (see Presets
   and holds). A system mode change
   goes through the same write gate as every other write (one write at a time, house-wide),
   so it can wait up to about 10 seconds; the card shows the requested mode meanwhile.
7. **Pause state survives a restart** of the ESP32, including a power cut, and is checked
   against the thermostat afterwards.
8. **The component never turns the switch on by itself**, and there are no timers: a pause
   lasts until it is ended.

Judging is done only from the thermostat's own register replies, and not for 30 seconds
after one of the component's own sends (its write is still being repeated and the reply
can lag). Nothing visible waits for that: the switch, the card and the Actual sensors
update at once.

## Presets and holds

The same component makes the thermostat card's presets and temperature edits behave like the
Carrier wall control, so the card works for day to day changes on its own.

### Presets

- **Per Schedule** releases any hold; the zone goes back to its schedule.
- **Wake, Home, Away, Sleep** send that activity's setpoints and fan. On a scheduled zone this
  first arms a hold until the next scheduled activity (exact, no minimum). On a timed hold the
  existing timer is kept, and a permanent hold stays permanent; no new hold is armed then.
- **Hold Indefinitely** is a real command: a permanent hold at the zone's current settings.
- **Hold Timer** is a readout, not a command. Tapping it writes nothing.
- **Vacation** is handled by InfinitESP itself and is unchanged here.
- **Paused** appears in the list while a zone is paused, and picking it pauses the zone (same as
  the switch). While paused, picking any other entry resumes to the remembered values with that
  hold: Hold Timer resumes exactly as the switch does, Hold Indefinitely resumes on a permanent
  hold, Per Schedule releases to the schedule, an activity applies that activity.
- While the system mode is Off, presets do nothing; the log says so. A temperature edit made
  while Off is kept as a desired value (below).

### Temperature edits

| Zone is... | An edit does... |
|---|---|
| Following its schedule | Arms a timed hold until the next scheduled activity (see `minimum_hold`), then sends the new setpoint(s) |
| On a timed hold | Sends the new setpoint(s); the timer is kept |
| On a permanent hold | Sends the new setpoint(s); stays permanent |

This mirrors the wall: a bump on a scheduled zone holds only until the next scheduled change.

### `minimum_hold`

A floor, in minutes, under the hold a temperature edit arms on a scheduled zone. Default 60,
range 0 to 1425; 0 behaves exactly like the wall, no minimum. It affects edits only; a preset
always holds for exactly the time left until the next scheduled activity.

```yaml
  - platform: zone_pause
    ...
    minimum_hold: 60   # minutes, 0-1425, default 60; 0 = no minimum (like the wall)
```

### Desired values

A setpoint side the current mode does not use (cool in Heat, heat in Cool, either side in Off)
cannot be written yet. The card shows it as a desired value and delivers it once the mode
changes to accept it. It is dropped instead when that side changes for any other reason (the wall, the
schedule, a newer edit) or after 24 hours, whichever comes first. A preset's undeliverable side is dropped right away, since a preset is a "do this
now" action.

### How the preset name is chosen

The card names an activity (Wake, Home, Away, Sleep) only when the zone's real setpoints, on
the side(s) the mode governs, plus the fan, match exactly one profile. The bus carries no
activity name; the schedule and clock break ties when more than one profile matches. No match
falls back to the hold kind (Hold Timer, Hold Indefinitely, Per Schedule) or Vacation.

### One write at a time

All zones, including fan changes, share one write gate: one write goes out at a time, about 10
seconds apart, house-wide. A burst of clicks does not burst writes: the first click's write
goes out at once, further clicks inside that gap only move the target, and one final write
follows once the gap opens. A newer preset or edit replaces an older one not yet sent. A preset
that changes the fan sends it as its own stage after the setpoints.

### Log and truth

Coalesced clicks and delivery failures are noted in the ESPHome log. What the thermostat
actually holds, independent of the card, is shown by the **Actual heat / cool setpoint
sensors** described above and by InfinitESP's own **Hold State** sensor, the truth channel for
the hold itself.

### Known limits of this feature

- The activity name never reaches the thermostat from this board. Elsewhere the same hold
  shows as a plain manual hold with an end time, not by name.
- InfinitESP's own hold-duration entities can collide with this feature's writes if used
  within about 10 seconds of a card action.
- Right after an edit, the preset name can briefly keep showing the previous activity.
- Unpausing while the bus is unreachable: the restore is queued and sent once the thermostat
  answers, and the pause switch reads off meanwhile.

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
    minimum_hold: 60           # optional, minutes 0-1425, default 60: floor under the hold a
                               # temperature edit arms on a scheduled zone (0 = like the wall)
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
- Every card action (setpoints, presets, hold commands, fan) is written by this component
  itself, in stages of one bus write each, through one write gate shared by all zones; only
  the system mode is forwarded to the wrapped entity, and that through the same gate.
- While a zone is paused, or still being put back, setpoint edits are written by this
  component itself rather than forwarded, because the wrapped entity would send its cached
  real value (a wide one) for the other setpoint.

## License

MIT. See [LICENSE](LICENSE) and [NOTICE](NOTICE). InfinitESP is the work of
[nebulous](https://github.com/nebulous) and is also MIT licensed.
