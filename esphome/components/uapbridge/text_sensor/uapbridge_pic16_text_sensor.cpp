#include "uapbridge_pic16_text_sensor.h"

namespace esphome {
namespace uapbridge_pic16 {
    static const char *const TAG = "uapbridge_pic16.text_sensor";
    void UAPBridgePic16TextSensor::setup() {
        this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
    }

    void UAPBridgePic16TextSensor::on_event_triggered() {
      if (this->parent_->get_state_string() != this->previousState_){
        this->previousState_ = this->parent_->get_state_string();
        ESP_LOGD(TAG, "UAPBridgePic16TextSensor::update() - %s", this->previousState_.c_str());
        this->publish_state(this->previousState_);
      }
    }

}  // namespace uapbridge_pic16
}  // namespace esphome