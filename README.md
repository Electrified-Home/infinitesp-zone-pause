# infinitesp-zone-pause

A per-zone **Pause** switch for Carrier Infinity / Bryant Evolution systems controlled
through [InfinitESP](https://github.com/nebulous/infinitesp).

This is a small ESPHome external component that sits in front of an InfinitESP zone.
It does not replace InfinitESP, and it never writes to the bus itself. It only calls the
InfinitESP hub's public methods. It does need a fork of InfinitESP (a hold-countdown fix plus
per-zone device support), pinned by commit: see
[The InfinitESP requirement](#the-infinitesp-requirement).

> **Status: experimental.** Compiles against ESPHome 2026.9. The pause switch passed live
> tests in the previous version of this component. This version changes how pause restores
> and cancels, and adds the shared target, Hold Minutes, Setting Status, waiting settings and
> per-zone devices; those are built and reviewed, but none of it has been run live. Read the
> [NOTICE](NOTICE).

## What it does

Pausing a zone stops it calling for heating or cooling by holding its setpoints wide
(default heat 50 °F / cool 85 °F) on a permanent hold. Turning pause off puts the zone
back the way it was. Typical use: a door or window is open, so a home automation hub
pauses the zones that would be heating or cooling the outdoors. The component knows nothing
about doors, windows or reasons: it offers one switch per zone, and your automations decide
when to use it.

Per zone you get:

| Entity | What it is |
|---|---|
| Thermostat (climate) | Always shows the **target** settings, never the wide pause values |
| Pause switch | On = paused, off = running. The handle for automations (the card also shows a Paused preset) |
| Actual heat / cool setpoint sensors (required) | What the thermostat really holds right now. The card never shows this, so these are the truth channel |
| Hold Minutes (optional number) | Minutes left on a timed hold (reads 0 for the schedule or a permanent hold; see InfinitESP's Hold State). Type any whole number of minutes to start a timed hold: the thermostat only takes 15-minute steps, so the board rounds up (under 15 becomes 15, 16 becomes 30). Type 0 for the schedule. The zone's temperatures stay as they are |
| Hold Until (optional sensor, read-only) | The clock time the target's timed hold ends, like `14:16` (unknown with no timed hold). It only shows the end; to change the hold, set Hold Minutes |
| Setting Status (optional text sensor) | What a waiting setting is waiting for, or why one was dropped |

These work with Home Assistant's standard thermostat card, toggle and number box; no custom
card is needed. The same component also makes the card's presets (Wake, Home, Away, Sleep,
Hold Indefinitely, Per Schedule) and temperature edits behave like the wall control, so no
Home Assistant scripts or helpers are needed to queue, delay or cancel a setting. See
[Presets and holds](#presets-and-holds).

## Install

Requires InfinitESP in active mode (`sam_address` not 0; the default, 0x92, makes InfinitESP
emulate a Carrier System Access Module, SAM). The examples assume the hub is
`id: infinitesp_hub`.

Give each InfinitESP zone an `id`, mark it `internal: true`, name it `"<Zone> Climate"`, and
let the wrapper take the zone's name. **The hidden block's name matters.** InfinitESP names
the zone's other entities (Temperature, Humidity, Fan Mode, ...) after that block and drops a
trailing "Climate". With `"Upstairs Climate"` they stay `Upstairs Temperature` and so on,
exactly as before. Any other name (say `"Upstairs Raw"`) renames every one of them in Home
Assistant.

Switch InfinitESP's own `hold_minutes` and `hold_until` off on the hidden block: they write
holds behind this component's back, and a timed hold set there ends a pause when it runs
out. This component's Hold Minutes replaces InfinitESP's Hold Minutes; give it the name
InfinitESP's had, `"<Zone> Hold Minutes"`, so its entity id does not change. Hold Until is
replaced the same way, by `"<Zone> Hold Until"`, now a read-only sensor.

```yaml
climate:
  - platform: infinitesp
    infinitesp_id: infinitesp_hub
    id: upstairs_raw
    name: "Upstairs Climate"   # see the note above
    internal: true
    zone: 1
    hold_minutes: false        # replaced by zone_pause's Hold Minutes
    hold_until: false          # replaced by zone_pause's Hold Until

  - platform: zone_pause
    infinitesp_id: infinitesp_hub
    source_id: upstairs_raw    # one zone_pause block per InfinitESP zone, never two
    name: "Upstairs"
    pause_heat_setpoint: 50    # optional, whole degrees FAHRENHEIT (40-99), default 50
    pause_cool_setpoint: 85    # same; at least 2 above the heat value
    minimum_hold: 60           # optional, minutes 0-1425, default 60 (see "At least 30 minutes")
    pause_switch:
      name: "Upstairs Pause"
    actual_heat_setpoint:      # required
      name: "Upstairs Actual Heat Setpoint"
    actual_cool_setpoint:      # required
      name: "Upstairs Actual Cool Setpoint"
    hold_minutes:              # optional
      name: "Upstairs Hold Minutes"
    hold_until:                # optional, read-only: the clock time the hold ends
      name: "Upstairs Hold Until"
    setting_status:            # optional
      name: "Upstairs Setting Status"
```

### The InfinitESP requirement

This component needs a fork of InfinitESP, and both components must be pinned by **commit
SHA**. Use this InfinitESP entry in place of any existing one; your other
`external_components` entries stay. For `zone_pause`, open this repository's Releases (or
Tags) page, find the tagged release `rev7.2` (or newer), and copy the commit SHA it points to into `ref`.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Electrified-Home/infinitesp
      ref: 74f6de2ae12a940871bae5cea8ed4f8b9a958866
  - source:
      type: git
      url: https://github.com/Electrified-Home/infinitesp-zone-pause
      ref: <commit of the rev7.2 tag>   # the full SHA from the rev7.2 tag, never a branch
    components: [zone_pause]
```

**Why the fork.** Every write InfinitESP sends (setpoint, fan, mode or hold) carries all
zones' running hold countdowns, copied from the thermostat's last reply, and the thermostat
floors a countdown to its 15-minute grid. Observed on setpoint writes: each one cut up to 14
minutes off a running timed hold, so a change could not be promised its 30 minutes. Fan and
mode writes send the same copied countdowns, so they most likely do the same; that is
inferred, not observed. The fork (`Electrified-Home/infinitesp`, branch `f1-echo-round-up`)
is upstream `4332fb8` plus two commits. The hold-countdown fix rounds the echoed countdowns
up to the next 15 minutes instead, so a running hold ends up to 14 minutes late, never early;
it has been proposed upstream. The other commit adds [per-zone devices](#per-zone-devices).

**Why by SHA.** This component reads InfinitESP's register layout and hub methods directly,
so an InfinitESP update may need a matching update here. A branch can move; a commit cannot.

## Per-zone devices

With the fork pinned above, each zone can be its own device in Home Assistant instead of
sharing the one InfinitESP device, so a script can target a zone (or an area you give it) and
every entity of that zone comes along. (An area also reaches other entities in it; target the
device to mean only the zone.) This needs ESPHome 2025.7 or later for `devices:` and Home
Assistant 2025.8 or later. On an existing install, read
[Updating an existing install](#updating-an-existing-install) before you flash.

Declare a device per zone, then put `device_id:` on both climate blocks and on each child
block of `zone_pause`. InfinitESP's other per-zone entities follow the hidden climate's
`device_id` themselves; a block without one, and the system-wide entities, stay on the main
device.

```yaml
esphome:
  devices:
    - id: zone_upstairs        # permanent: Home Assistant keys the device on this text
      name: "Upstairs"
climate:
  - platform: infinitesp
    ...
    device_id: zone_upstairs
  - platform: zone_pause
    ...
    device_id: zone_upstairs
    pause_switch:
      device_id: zone_upstairs   # likewise on actual_heat_setpoint, actual_cool_setpoint,
      name: "Upstairs Pause"     # hold_minutes, hold_until and setting_status
```

Existing entities keep their names. With names unchanged, Home Assistant's code (2025.8 or
later) moves an entity onto its new device and keeps its entity id. That is what the code
does, not a guarantee, so follow the steps in
[Updating an existing install](#updating-an-existing-install). Three things to expect:

- **The area is lost.** Moved entities do not keep the host device's area. Assign each zone
  device an area in Home Assistant.
- **Friendly names may show the zone name twice.** Home Assistant puts the device name in
  front of an entity's name, so a name that already starts with the zone name can read
  doubled. A new entity can take a short name such as `"Setting Status"` instead.
- **Leaving devices is destructive.** See the last rule under Updating.

## Updating an existing install

Read this before flashing an update to a running install, above all when adding `devices:`.

- **Turn every pause switch off before flashing**, and wait until each zone shows its normal
  Actual Heat and Actual Cool values again (the restore takes about a minute). An update
  that changes the saved record forgets a pause while the thermostat keeps the zone wide
  (adding devices changes it too). A pause that was forgotten can leave the zone at the wide
  values (50 / 85 by default) with the switch off; the boot log then reports that the saved
  pause state is from another version, and you set the temperatures by hand.
- **Do not rename any entity in the same flash.** Home Assistant follows an entity onto its
  new device by name.
- **Adding devices: take a Home Assistant backup first** (ESPHome 2025.7 or later, Home
  Assistant 2025.8 or later).
- **The first devices flash needs `esp32: framework: advanced: enable_ota_rollback:
  false`.** Otherwise a crash or power loss in the board's first minute makes the ESP32 fall
  back to the previous firmware, which has no devices, and Home Assistant then deletes the
  zone devices' entities.
- **Soon after the first devices flash, flash again without that line** (rollback back on,
  the default). Until you do, later updates have no automatic fall-back.
- **After the flash, check one zone before relying on the rest.** Its device should appear
  and its entities should keep their entity ids, with no duplicate (`_2`) entity or orphan.
  If the ids did change, restore the Home Assistant backup. If they held, check the other
  zones and assign each zone device an area.
- **Do not remove `devices:` or `device_id` later.** Removing them (for example by flashing
  an older config that does not have them) makes Home Assistant delete the zone devices'
  entities. Use the same safe steps in the reverse direction, or don't do it.

## What we learned about the thermostat

Observed on one Infinity Touch system; a starting point, not a specification:

| System mode | Heat setpoint write | Cool setpoint write |
|---|---|---|
| Heat | accepted | **ignored** |
| Cool | not tested | accepted |
| Auto | not tested | not tested |
| Off | **ignored** | **ignored** |

The thermostat only takes the setpoint its current mode uses, and nothing while off; a mode
change does not alter setpoints. InfinitESP repeats each write three times, the thermostat's
reply can lag by several seconds, and it drops a write that follows another within a fraction
of a second. So this component never sends two writes back to back, and it judges only from
the thermostat's own register replies, never from the hub's optimistic copy of a command.

## Behavior

1. **Pause is an indefinite hold on the zone.** Switch on: remember the zone's target (its
   setpoints and its hold, the snapshot), send both wide setpoints and a permanent hold. The
   zone is paused at once, in any system mode, even when the thermostat will not take the
   values yet, and stays paused until it is ended: nothing times out, and the component never
   turns the switch on by itself.
2. **A system mode change while paused keeps the pause** and sends again, so the side the
   new mode uses goes wide when the mode allows it (off to heat, heat to cool, and so on).
3. **Three deliberate things end a pause** ([Known limits](#known-limits) lists accidental
   ones): the switch is turned off (the snapshot is put back); a preset is picked on the
   card; or a setpoint the current mode uses is changed by anything else (wall control, the
   Carrier app, another integration) to something other than its pause value, and that value
   stands. Heat mode counts only the heat setpoint, cool mode only the cool setpoint, auto
   and off both; a change to the other setpoint becomes that side's new target and the pause
   carries on.
4. **A send that does not land is repeated once**, when the thermostat's reply shows the
   value did not arrive. Then the component gives up and logs an error, and the card shows
   what the thermostat really holds. A pause only stops resending: it stays paused.
5. **Putting the zone back** returns what is still valid: a zone that followed its schedule
   gets its values back and the hold released; a timed hold still running comes back to the
   end time it had (a restart or a long pause does not change it); a timed hold that ran out
   during the pause is not put back, and the zone returns to its schedule; a permanent hold
   comes back with the setpoints. A part the current mode will not take is delivered when
   the mode allows; the card keeps showing the target meanwhile.
6. **Mode and fan are not affected by pause.** While a zone is paused its fan is forwarded
   straight to the hub, with no gate. Every other write, the system mode included, goes
   through one write gate. All zones share it: one write at a time, about 10 seconds apart,
   house-wide, so a mode change can wait up to about 10 seconds (the card shows the
   requested mode meanwhile). The first click's write goes out at once, further clicks inside
   the gap only move the target, and one final write follows. A change that needs a hold goes
   out as stages, one write each: the hold, then the setpoints, then the fan if a preset
   changes it. Merged clicks, ignored requests and failed deliveries are noted in the ESPHome
   log.
7. **Pause state and the zone's target survive a restart** of the ESP32, including a power
   cut, and are checked against the thermostat afterwards.

Three lines in the ESPHome log mark a pause: `PAUSE. Remembering H / C, ...` when it starts,
`UNPAUSE. Putting back H / C, ...` when the switch or a preset ends it, and (a warning)
`PAUSE CANCELLED: something else changed the setpoints to H / C. The zone is no longer paused
and those values stand.` when a change from outside ends it. Then the switch reads off; a
side still at its wide value is put back to the remembered target when the mode takes it.

## Presets and holds

The card's presets and temperature edits behave like the Carrier wall control, so the card
works for day to day changes on its own.

### One target per zone

Each zone has one target: the setpoints it should have, and until when (its schedule, a hold
until a time, or a permanent hold). What a pause remembers and a later request are the same
thing, the newest target: the latest request wins, and a change made anywhere else (the
wall, the Carrier app, another integration, the schedule's own new values) replaces it. The
hold the thermostat shows is the truth. When the zone can take the target (not paused, in a
mode that accepts it) it is applied if still valid, otherwise the schedule applies. The
standard controls work in any pause state and system mode (presets in Off excepted):

```yaml
# Heat 68 / cool 76 for 90 minutes (in your Home Assistant's units). Use your own entity ids.
- action: climate.set_temperature
  target: { entity_id: climate.upstairs }
  data: { target_temp_low: 68, target_temp_high: 76 }
- action: number.set_value
  target: { entity_id: number.upstairs_hold_minutes }
  data: { value: 90 }
```

### Presets

- **Per Schedule** releases any hold and drops any waiting setting; the zone goes back to
  its schedule.
- **Wake, Home, Away, Sleep** send that activity's setpoints and fan. On a scheduled zone this
  first arms a hold until the next scheduled activity; a timed hold keeps its end and a
  permanent hold stays permanent.
- **Hold Indefinitely** is a real command: a permanent hold at the zone's current settings.
  **Hold Timer** is a readout: tapping it writes nothing.
- **Vacation** is shown while the thermostat is in vacation mode. Picking it does nothing,
  paused or not: a pause carries on.
- **Paused** is always in the list and is what the card shows while the zone is paused.
  Picking it pauses the zone (same as the switch). Picking any other entry except Vacation
  ends the pause and applies that entry (Hold Timer resumes exactly as the switch does).
- While the system mode is Off, the Wake, Home, Away and Sleep presets are not applied (the
  log says so); Per Schedule and Hold Indefinitely wait for the system to turn on.

The bus carries no activity name, so the card works it out: Paused or Vacation first, else an
activity if a profile matches the real setpoints and fan, else Hold Timer, Hold Indefinitely
or Per Schedule. When several profiles match, a zone on a hold shows the hold instead; with
no hold, the schedule's current activity breaks the tie, and when that is not one of them
the first matching profile shows. Elsewhere (the wall control, the Carrier app) the same hold
shows as a plain manual hold, not by name.

### Temperature edits

| Zone is... | An edit does... |
|---|---|
| Following its schedule | Arms a timed hold until the next scheduled activity, then sends the new setpoint(s) |
| On a timed hold | Sends the new setpoint(s); the hold's end is kept |
| On a permanent hold | Sends the new setpoint(s); stays permanent |
| Paused | Sends nothing; the edit goes into the remembered target (see below) |

### At least 30 minutes

Whatever is sent lasts at least 30 minutes: a hold armed for a change is never shorter, a
timed hold with under about 23 minutes left is stretched to 30, and one with more keeps its
end. A restore is not a change; it puts back the time that was left. Nor is an explicit
Hold Minutes: it is honoured as typed, rounded up to 15 (Hold Minutes 15 holds 15 minutes). `minimum_hold` is a
further floor under the hold an edit arms on a scheduled zone (0 = only the 30 minutes); it
does not affect presets. Holds are written to the nearest 15 minutes and the thermostat
counts them on that grid, so a hold can end a few minutes either side of the minute asked
for; the pinned InfinitESP only ever rounds a running countdown up, never early.

### Special cases of edits

- **Paused:** an edit changes the remembered target and sends nothing. It counts as a change
  with an end (an existing hold keeps its end; a scheduled zone holds until the next
  scheduled activity, or N minutes with Hold Minutes N). It goes out at the unpause with
  whatever time is left; if the pause outlasts the end, it is dropped and the schedule returns.
- **A setting the current mode cannot take** (cool in Heat, heat in Cool, either side in
  Off) waits in the target: the card shows it, Setting Status says what it waits for, and it
  is delivered once the mode accepts it. It rides the hold that is already there and ends
  with it: nothing is written for it alone, and it gets no 30-minute minimum (use Hold
  Minutes to ask for longer). It is dropped, and Setting Status says why, when its end
  passes or anything else changes the zone's hold or setpoints; a newer request from the
  card simply replaces it. A preset's undeliverable side is dropped at once.
- **Invalid pairs are made valid**, as the wall does: with heat at 70, setting cool to 68
  moves heat down to keep the thermostat's own 2 degree gap. The side just set wins (with
  both given, the one that moved further). Values stay between 40 and 99 °F.
- **A request that matches what will happen anyway sends nothing**: the value already in
  force or waiting, the value an unpause will put back, or the hold already running.

### Hold Minutes

The optional Hold Minutes number is described above. Writing **0** returns the zone to its
schedule (a temperature given just before it is dropped, as Per Schedule does); writing
**N** (any whole number up to 1425) makes a timed hold of N minutes from now, rounded UP to
the thermostat's 15-minute steps, at least 15 (5 and 15 both give 15; 16 gives 30). The
30-minute minimum is for the hold a temperature edit starts by itself, not for Hold Minutes.
A Hold Minutes alone is sent: the hold is written first and, about 10 seconds later, the
zone's own setpoints are written back, because a timed-hold write resets the zone to its
schedule values. Given with a temperature, in either order, the two form one target.
While paused, or in Off, it goes into the target and waits.

### Setting Status

The optional text sensor shows one of these:

| Text | Meaning |
|---|---|
| `Nothing waiting` | Nothing is pending |
| `Waiting for the pause to end` | The zone is paused; the target goes out at the unpause |
| `Waiting for the system to turn on` | A setting waits and the system mode is Off |
| `Waiting for heat or auto mode` | A heat setting waits in Cool mode |
| `Waiting for cool or auto mode` | A cool setting waits in Heat mode |
| `Dropped: the setting's time ran out` | The waiting setting's end passed (paused: the target's hold ended during the pause, so the unpause goes to the schedule) |
| `Dropped: the zone's hold ended` | The zone's own timed hold ran out, so the waiting setting no longer matters |
| `Dropped: the zone was changed elsewhere` | The wall, the app, another integration or the schedule changed the zone |
| `Dropped: the thermostat did not accept the change` | The component gave up after its one repeat |

Either of the first two `Dropped` lines may show when a hold runs out. A `Dropped` line stays
until the next accepted change, pause or unpause.

## Known limits

- When a pause cannot start (bus offline, no recent answer from the thermostat) or is
  cancelled, the switch simply reads off; the reason is in the ESPHome log.
- A change made at the wall in the first seconds after a pause or unpause can be overwritten
  once by the repeats of the component's own write; made again, it counts. Noticing a change
  can take up to 30 seconds right after a send, a few seconds otherwise.
- A pause cancelled by a change at the wall on a zone that was following its schedule leaves
  the pause's permanent hold in place (releasing it would discard the value just set).
- Untested: starting Vacation changes the zone's setpoints, which the pause reads as a change
  from outside, so it will most likely cancel the pause.
- A restart is a small gap: a change made at the wall while the board was down is not seen
  under a waiting setting, and a pause taken while the system is Off, then restarted after
  the schedule moved to a new period, can read as PAUSE CANCELLED.
- A change that needs a timed hold is refused (error in the log) until the thermostat's
  clock has been read, in the first seconds after boot. If a zone's schedule cannot be
  read, "until the next scheduled activity" becomes `minimum_hold` (or 60) minutes.
- The component assumes the thermostat's 2 degree heat/cool gap (the installer default); a
  different setting needs `SETPOINT_GAP` in the code changed to match.
- A paused zone still heats below the wide heat value and cools above the wide cool value.
  That is intended: it is the freeze guard.
- In a rare sequence of three edits within 30 seconds, a hold can run 45 minutes instead of
  60. It is always a timed hold, never permanent.

## License

MIT. See [LICENSE](LICENSE) and [NOTICE](NOTICE). InfinitESP is the work of
[nebulous](https://github.com/nebulous) and is also MIT licensed.
