from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import (
    CONF_ID,
    CONF_NUMBER,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_MODE,
    CONF_INVERTED,
    CONF_BAUD_RATE,
)

# === NAMESPACE ===
max14830_ns = cg.esphome_ns.namespace("max14830")
MAX14830 = max14830_ns.class_("MAX14830", cg.Component, spi.SPIDevice)
MAX14830GPIOPin = max14830_ns.class_("MAX14830GPIOPin", cg.GPIOPin)
MAX14830UART = max14830_ns.class_("MAX14830UART", cg.Component)

# === CONFIG ===
CONF_MAX14830 = "max14830"
CONF_PORT = "port"

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.declare_id(MAX14830),
}).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema())

CODEOWNERS = []
DEPENDENCIES = ["spi"]

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await cg.register_component(var, config)

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

# === UART SUPPORT ===
MAX14830_UART_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.declare_id(MAX14830UART),
    cv.Required(CONF_MAX14830): cv.use_id(MAX14830),
    cv.Required(CONF_PORT): cv.int_range(min=0, max=3),
    cv.Optional(CONF_BAUD_RATE, default=9600): cv.positive_int,
})

CONFIG_SCHEMA = cv.All(
    CONFIG_SCHEMA.extend({
        cv.Optional("max14830_uart"): cv.ensure_list(MAX14830_UART_SCHEMA),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await cg.register_component(var, config)

    for uart_cfg in config.get("max14830_uart", []):
        uart = cg.new_Pvariable(uart_cfg[CONF_ID])
        cg.add(uart.set_parent(var))
        cg.add(uart.set_port(uart_cfg[CONF_PORT]))
        cg.add(uart.set_baud_rate(uart_cfg[CONF_BAUD_RATE]))
        await cg.register_component(uart, uart_cfg)
