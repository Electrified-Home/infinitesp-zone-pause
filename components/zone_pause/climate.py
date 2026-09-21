import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor, switch
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
AUTO_LOAD = ["sensor", "switch"]

CONF_SOURCE_ID = "source_id"
CONF_PAUSE_HEAT_SETPOINT = "pause_heat_setpoint"
CONF_PAUSE_COOL_SETPOINT = "pause_cool_setpoint"
CONF_PAUSE_SWITCH = "pause_switch"
CONF_ACTUAL_HEAT_SETPOINT = "actual_heat_setpoint"
CONF_ACTUAL_COOL_SETPOINT = "actual_cool_setpoint"

zone_pause_ns = cg.esphome_ns.namespace("zone_pause")
# Not a Component on purpose: it registers with the InfinitESP hub as one of its
# lightweight entities instead (see zone_pause.h).
ZonePauseClimate = zone_pause_ns.class_("ZonePauseClimate", climate.Climate, InfinitESPEntity)
ZonePauseSwitch = zone_pause_ns.class_("ZonePauseSwitch", switch.Switch)


def _validate(config):
    # Carrier keeps heat and cool at least 2 degrees apart (MIN_GAP in zone_pause.cpp).
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
            cv.Required(CONF_PAUSE_SWITCH): switch.switch_schema(
                ZonePauseSwitch, icon="mdi:pause-circle-outline"
            ),
            # Required: the thermostat card always shows the target, so these two are the only
            # place in Home Assistant that shows what the thermostat really holds.
            cv.Required(CONF_ACTUAL_HEAT_SETPOINT): _setpoint_sensor_schema(),
            cv.Required(CONF_ACTUAL_COOL_SETPOINT): _setpoint_sensor_schema(),
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

    sw = await switch.new_switch(config[CONF_PAUSE_SWITCH])
    cg.add(sw.set_parent(var))
    cg.add(var.set_pause_switch(sw))

    sens = await sensor.new_sensor(config[CONF_ACTUAL_HEAT_SETPOINT])
    cg.add(var.set_actual_heat_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_ACTUAL_COOL_SETPOINT])
    cg.add(var.set_actual_cool_sensor(sens))

    cg.add(var.init())
