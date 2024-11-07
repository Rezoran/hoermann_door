#include "uapbridge_pic16_binaryOutput.h"

#include "esphome/core/log.h"

namespace esphome {
namespace uapbridge_pic16 {

static const char *TAG = "uapbridge_pic16.binary_output";

void UAPBridge_pic16BinaryOutput::write_state(bool state) {
  if (this->parent_->get_light_enabled() != state) {
    ESP_LOGD(TAG,
             "UAPBridge_pic16BinaryOutput::write_state() - setting light to %s",
             state ? "true" : "false");
    this->parent_->action_toggle_light();
  }
}

void UAPBridge_pic16BinaryOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge_pic16BinaryOutput:");
  if (this->parent_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Parent UAPBridge_pic16 is set.");
  } else {
    ESP_LOGCONFIG(TAG, "  Warning: Parent UAPBridge_pic16 is not set.");
  }
}

}  // namespace uapbridge_pic16
}  // namespace esphome
