from esphome.components import cover
import esphome.config_validation as cv
import esphome.codegen as cg
from .. import uapbridge_pic16_ns, CONF_UAPBRIDGE_PIC16_ID, UAPBridge_pic16
from esphome.const import (
  CONF_ID
)

DEPENDENCIES = ["uapbridge_pic16"]

UAPBridgePic16Cover = uapbridge_pic16_ns.class_("UAPBridge_pic16Cover", cover.Cover, cg.Component)

# Use COVER_SCHEMA instead of cover_schema
CONFIG_SCHEMA = (
   cover.COVER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(UAPBridgePic16Cover),
            cv.GenerateID(CONF_UAPBRIDGE_PIC16_ID): cv.use_id(UAPBridge_pic16),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    # Create a new Pvariable for UAPBridgePic16Cover
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    # Retrieve the parent variable
    parent = await cg.get_variable(config[CONF_UAPBRIDGE_PIC16_ID])
    cg.add(var.set_uapbridge_pic16_parent(parent))

