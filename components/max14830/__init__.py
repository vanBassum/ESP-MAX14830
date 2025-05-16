from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi, uart
from esphome.const import (
    CONF_ID,
    CONF_NUMBER,
    CONF_BAUD_RATE,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_MODE,
    CONF_INVERTED,
)

# === NAMESPACE ===
max14830_ns = cg.esphome_ns.namespace("max14830")
MAX14830 = max14830_ns.class_("MAX14830", cg.Component, spi.SPIDevice)
MAX14830GPIOPin = max14830_ns.class_("MAX14830GPIOPin", cg.GPIOPin)
MAX14830UART = max14830_ns.class_("MAX14830UART", cg.Component)

# === CONSTANTS ===
CONF_MAX14830 = "max14830"
CONF_UART = "uart"

# === METADATA ===
CODEOWNERS = []
DEPENDENCIES = ["spi", "uart"]
AUTO_LOAD = ["uart"]

# === UART Schema ===
UART_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.declare_id(MAX14830UART),
    cv.Required(CONF_NUMBER): cv.int_range(min=0, max=3),
    cv.Required(CONF_BAUD_RATE): cv.positive_int
})

# === Main Schema ===
CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.Required(CONF_ID): cv.declare_id(MAX14830),
        cv.Optional(CONF_UART): cv.ensure_list(UART_SCHEMA),
    }).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema()),
    lambda value: _validate_uart_channels(value, 4),
)


def _validate_uart_channels(value, max_channels):
    seen = set()
    for uart_cfg in value.get(CONF_UART, []):
        ch = uart_cfg[CONF_NUMBER]
        if ch in seen:
            raise cv.Invalid(f"Duplicate UART channel: {ch}")
        if ch >= max_channels:
            raise cv.Invalid(f"UART channel out of range (0-{max_channels - 1}): {ch}")
        seen.add(ch)
    return value

# === COMPONENT CREATION ===
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await cg.register_component(var, config)

    for uart_cfg in config.get(CONF_UART, []):
        uart_var = cg.new_Pvariable(uart_cfg[CONF_ID])
        cg.add(uart_var.set_parent(var))
        cg.add(uart_var.set_port(uart_cfg[CONF_NUMBER]))
        cg.add(uart_var.set_baud_rate(uart_cfg[CONF_BAUD_RATE]))
        await cg.register_component(uart_var, uart_cfg)

# === GPIO PIN SUPPORT ===
def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value

MAX14830_PIN_SCHEMA = pins.gpio_base_schema(
    MAX14830GPIOPin,
    cv.int_range(min=0, max=15),
    modes=[CONF_INPUT, CONF_OUTPUT],
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
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var
