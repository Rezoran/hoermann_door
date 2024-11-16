from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from .. import uapbridge_pic16_ns, CONF_UAPBRIDGE_PIC16_ID, UAPBridge_pic16

DEPENDENCIES = ["uapbridge_pic16"]

UAPBridgePic16TextSensor = uapbridge_pic16_ns.class_("UAPBridgePic16TextSensor", text_sensor.TextSensor, cg.Component)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(UAPBridgePic16TextSensor)
    .extend(
        {
            cv.GenerateID(CONF_UAPBRIDGE_PIC16_ID): cv.use_id(UAPBridge_pic16),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_UAPBRIDGE_PIC16_ID])
    cg.add(var.set_uapbridge_pic16_parent(parent))
