from esphome.components import output
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from .. import uapbridge_pic16_ns, UAPBridge_pic16, CONF_UAPBRIDGE_PIC16_ID

DEPENDENCIES = ["uapbridge_pic16"]

UAPBridge_pic16BinaryOutput = uapbridge_pic16_ns.class_("UAPBridge_pic16BinaryOutput", output.BinaryOutput, cg.Component)

CONFIG_SCHEMA = output.BINARY_OUTPUT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(UAPBridge_pic16BinaryOutput),
        cv.GenerateID(CONF_UAPBRIDGE_PIC16_ID): cv.use_id(UAPBridge_pic16),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await output.register_output(var, config)

    parent = await cg.get_variable(config[CONF_UAPBRIDGE_PIC16_ID])
    cg.add(var.set_uapbridge_pic16_parent(parent))
