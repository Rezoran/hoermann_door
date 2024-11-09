import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome import pins
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

CONF_RTS_PIN = "rts_pin"
CONF_AUTO_CORRECTION = "auto_correction"

# Create UAPBridge namespace
uapbridge_ns = cg.esphome_ns.namespace("uapbridge")
UAPBridge = uapbridge_ns.class_("UAPBridge", cg.Component, uart.UARTDevice)

CONF_UAPBRIDGE_ID = "uapbridge_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UAPBridge),
        cv.Optional(CONF_RTS_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_AUTO_CORRECTION): cv.boolean,
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "uapbridge_uart",
    require_tx=True,
    require_rx=True,
    baud_rate=19200,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RTS_PIN in config:
        rts_pin = await cg.gpio_pin_expression(config[CONF_RTS_PIN])
        cg.add(var.set_rts_pin(rts_pin))

    if CONF_AUTO_CORRECTION in config:
        cg.add(var.set_auto_correction(config[CONF_AUTO_CORRECTION]))
