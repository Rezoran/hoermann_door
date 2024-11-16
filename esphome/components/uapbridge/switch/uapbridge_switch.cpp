#include "uapbridge_pic16_switch.h"

namespace esphome {
namespace uapbridge_pic16 {
static const char *const TAG = "uapbridge_pic16.switch";
void UAPBridge_pic16SwitchVent::setup() {
    this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}
void UAPBridge_pic16SwitchVent::on_event_triggered() {
  if (this->parent_->get_venting_enabled() != this->previousState_) {
    ESP_LOGD(TAG, "UAPBridge_pic16SwitchVent::on_event_triggered() - adjusting state");
    this->publish_state(this->parent_->get_venting_enabled());
    this->previousState_ = this->parent_->get_venting_enabled();
  }
}

void UAPBridge_pic16SwitchVent::write_state(bool state) {
  UAPBridge_pic16::hoermann_state_t current_state = this->parent_->get_state();

  if (state && current_state != UAPBridge_pic16::hoermann_state_t::hoermann_state_venting) {
    ESP_LOGD(TAG, "UAPBridge_pic16SwitchVent::write_state() - Setting door to vent");
    this->parent_->set_venting(state);
  } else if (!state && current_state != UAPBridge_pic16::hoermann_state_t::hoermann_state_closed) {
    ESP_LOGD(TAG, "UAPBridge_pic16SwitchVent::write_state() - Closing door");
    this->parent_->set_venting(state);
  } else {
    ESP_LOGD(TAG, "UAPBridge_pic16SwitchVent::write_state() - Door already in desired state");
  }
}

void UAPBridge_pic16SwitchVent::dump_config() {
    ESP_LOGCONFIG(TAG, "UAPBridge_pic16SwitchVent");
}

void UAPBridge_pic16SwitchLight::setup() {
    this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}
void UAPBridge_pic16SwitchLight::on_event_triggered() {
  if (this->parent_->get_light_enabled() != this->previousState_) {
    ESP_LOGD(TAG, "UAPBridge_pic16SwitchLight::on_event_triggered() - adjusting state");
    this->publish_state(this->parent_->get_light_enabled());
    this->previousState_ = this->parent_->get_light_enabled();
  }
}
void UAPBridge_pic16SwitchLight::write_state(bool state) {
  ESP_LOGD(TAG, "UAPBridge_pic16SwitchLight::write_state() - write State triggered");
  if (this->parent_->get_light_enabled() != state){
    this->parent_->action_toggle_light();
  }
  //@TODO Check if make sens or not
  publish_state(state);
}
void UAPBridge_pic16SwitchLight::dump_config() {
    ESP_LOGCONFIG(TAG, "UAPBridge_pic16SwitchLight");
}
}
}