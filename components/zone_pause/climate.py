import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, number, sensor, switch, text_sensor
from esphome.components.infinitesp import (
    CONF_INFINITESP_ID,
    InfinitESPComponent,
    InfinitESPEntity,
    register_infinitesp_entity,
)
from esphome.components.infinitesp.climate import InfinitESPClimate
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["infinitesp"]
AUTO_LOAD = ["number", "sensor", "switch", "text_sensor"]

CONF_SOURCE_ID = "source_id"
CONF_PAUSE_HEAT_SETPOINT = "pause_heat_setpoint"
CONF_PAUSE_COOL_SETPOINT = "pause_cool_setpoint"
CONF_MINIMUM_HOLD = "minimum_hold"
CONF_PAUSE_SWITCH = "pause_switch"
CONF_ACTUAL_HEAT_SETPOINT = "actual_heat_setpoint"
CONF_ACTUAL_COOL_SETPOINT = "actual_cool_setpoint"
CONF_HOLD_MINUTES = "hold_minutes"
CONF_SETTING_STATUS = "setting_status"

zone_pause_ns = cg.esphome_ns.namespace("zone_pause")
# Not a Component on purpose: it registers with the InfinitESP hub as one of its
# lightweight entities instead (see zone_pause.h).
ZonePauseClimate = zone_pause_ns.class_("ZonePauseClimate", climate.Climate, InfinitESPEntity)
ZonePauseSwitch = zone_pause_ns.class_("ZonePauseSwitch", switch.Switch)
ZonePauseHoldMinutes = zone_pause_ns.class_("ZonePauseHoldMinutes", number.Number)


def _validate(config):
    # Carrier keeps heat and cool at least 2 degrees apart; the wide pair must respect that too.
    if config[CONF_PAUSE_HEAT_SETPOINT] + 2 > config[CONF_PAUSE_COOL_SETPOINT]:
        raise cv.Invalid("pause_heat_setpoint must be at least 2 below pause_cool_setpoint")
    return config


def _setpoint_sensor_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=1,
    )


CONFIG_SCHEMA = cv.All(
    climate.climate_schema(ZonePauseClimate).extend(
        {
            cv.GenerateID(CONF_INFINITESP_ID): cv.use_id(InfinitESPComponent),
            cv.Required(CONF_SOURCE_ID): cv.use_id(InfinitESPClimate),
            # Whole degrees FAHRENHEIT, the unit the Carrier bus works in (not Celsius).
            cv.Optional(CONF_PAUSE_HEAT_SETPOINT, default=50): cv.int_range(min=40, max=99),
            cv.Optional(CONF_PAUSE_COOL_SETPOINT, default=85): cv.int_range(min=40, max=99),
            # A temperature change sent at once to a zone following its schedule holds until the
            # next scheduled activity (as the wall does), but never less than this many minutes
            # nor less than 30; the hub rounds that hold to 15. One that waits (paused, or a side
            # the mode cannot take yet) holds exactly to the next activity, unrounded. 0 = no
            # minimum beyond the 30. Whole minutes.
            cv.Optional(CONF_MINIMUM_HOLD, default=60): cv.int_range(min=0, max=1425),
            cv.Required(CONF_PAUSE_SWITCH): switch.switch_schema(
                ZonePauseSwitch, icon="mdi:pause-circle-outline"
            ),
            # Required: the thermostat card always shows the target, so these two are the only
            # place in Home Assistant that shows what the thermostat really holds.
            cv.Required(CONF_ACTUAL_HEAT_SETPOINT): _setpoint_sensor_schema(),
            cv.Required(CONF_ACTUAL_COOL_SETPOINT): _setpoint_sensor_schema(),
            # Optional: how long the target holds, read and set (the zone's own Hold Minutes, in
            # place of InfinitESP's: turn that one off with hold_minutes: false and
            # hold_until: false on the hidden block). 0 = the schedule.
            cv.Optional(CONF_HOLD_MINUTES): number.number_schema(
                ZonePauseHoldMinutes, unit_of_measurement="min", icon="mdi:timer-outline"
            ),
            # Optional: what the target is waiting for, or why a waiting one was dropped.
            cv.Optional(CONF_SETTING_STATUS): text_sensor.text_sensor_schema(),
        }
    ),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)
    await register_infinitesp_entity(var, config)

    source = await cg.get_variable(config[CONF_SOURCE_ID])
    cg.add(var.set_source(source))
    cg.add(var.set_pause_setpoints(config[CONF_PAUSE_HEAT_SETPOINT], config[CONF_PAUSE_COOL_SETPOINT]))
    cg.add(var.set_minimum_hold(config[CONF_MINIMUM_HOLD]))

    sw = await switch.new_switch(config[CONF_PAUSE_SWITCH])
    cg.add(sw.set_parent(var))
    cg.add(var.set_pause_switch(sw))

    sens = await sensor.new_sensor(config[CONF_ACTUAL_HEAT_SETPOINT])
    cg.add(var.set_actual_heat_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_ACTUAL_COOL_SETPOINT])
    cg.add(var.set_actual_cool_sensor(sens))

    if CONF_HOLD_MINUTES in config:
        # The thermostat's timed-hold range and grid, as InfinitESP's own Hold Minutes.
        num = await number.new_number(config[CONF_HOLD_MINUTES], min_value=0, max_value=1425, step=15)
        cg.add(num.set_parent(var))
        cg.add(var.set_hold_minutes_number(num))
    if CONF_SETTING_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SETTING_STATUS])
        cg.add(var.set_setting_status_sensor(sens))

    cg.add(var.init())
