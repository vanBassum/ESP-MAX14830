from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import (
    CONF_ID,
    CONF_NUMBER,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_PULLUP,
    CONF_MODE,
    CONF_INVERTED,
)
from esphome.core import coroutine

# === NAMESPACE ===
max14830_ns = cg.esphome_ns.namespace("max14830")
MAX14830 = max14830_ns.class_("MAX14830", spi.SPIDevice, cg.Component)
MAX14830GPIOPin = max14830_ns.class_("MAX14830GPIOPin", cg.GPIOPin)

# === YAML CONFIG FOR MAIN COMPONENT ===
CONF_MAX14830 = "max14830"
CONF_DEVICE_ADDRESS = "device_address"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MAX14830),
    cv.Optional(CONF_DEVICE_ADDRESS, default=0): cv.uint8_t,
}).extend(spi.spi_device_schema())

CODEOWNERS = []
DEPENDENCIES = ["spi"]
AUTO_LOAD = []

@coroutine
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    cg.add(var.set_device_address(config[CONF_DEVICE_ADDRESS]))
    await cg.register_component(var, config)

# === PIN SUPPORT ===

def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_PULLUP] and not value[CONF_INPUT]:
        raise cv.Invalid("Pullup only available with input")
    return value

MAX14830_PIN_SCHEMA = pins.gpio_base_schema(
    MAX14830GPIOPin,
    cv.int_range(min=0, max=15),
    modes=[CONF_INPUT, CONF_OUTPUT, CONF_PULLUP],
    mode_validator=validate_mode,
    invertable=True,
).extend({
    cv.Required(CONF_MAX14830): cv.use_id(MAX14830),
})

@pins.PIN_SCHEMA_REGISTRY.register(CONF_MAX14830, MAX14830_PIN_SCHEMA)
async def max14830_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_MAX14830])
    cg.add(var.set_parent(parent))
    cg.add(var.set_pin(config[CONF_NUMBER]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    return var
