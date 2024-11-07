#include "uapbridge_pic16_cover.h"
namespace esphome {
namespace uapbridge_pic16 {

static const char* const TAG = "uapbridge_pic16.cover";

void UAPBridge_pic16Cover::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}

cover::CoverTraits UAPBridge_pic16Cover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  //  traits.set_supports_stop(true); //PIC16 generate an error on the motor by sending this command
  traits.set_supports_tilt(false);
  return traits;
}

void UAPBridge_pic16Cover::control(const cover::CoverCall& call) {
  if (call.get_position().has_value()) {
    float position = *call.get_position();
    if (position == 0) {
      // Close the cover
      parent_->action_close();
      ESP_LOGI(TAG, "Closing the cover");
    } else if (position == 1) {
      // Open the cover
      parent_->action_open();
      ESP_LOGI(TAG, "Opening the cover");
    }
  }
  if (call.get_stop()) {
    // Stop the cover
    parent_->action_stop();
    ESP_LOGI(TAG, "Stopping the cover");
  }
}

void UAPBridge_pic16Cover::on_event_triggered() {
  switch (this->parent_->get_state()) {
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_opening:
      this->current_operation = cover::COVER_OPERATION_OPENING;
      break;
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_open:
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->position = 1;
      break;
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_closed:
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->position = 0;
      break;
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_error:
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_unkown:
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_venting:
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_stopped:
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->position = 0.1;
      break;
    case UAPBridge_pic16::hoermann_state_t::hoermann_state_closing:
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      break;
  }
  if (this->previousState_ != this->parent_->get_state() || this->previousOperation_ != this->current_operation) {
    ESP_LOGV(TAG, "HCPBridgeCover::update() - position is %f", this->position);
    ESP_LOGV(TAG, "HCPBridgeCover::update() - operation is %d", this->current_operation);
    ESP_LOGD(TAG, "HCPBridgeCover::update() - state changed");
    this->publish_state(false);
    this->previousState_ = this->parent_->get_state();
    this->previousOperation_ = this->current_operation;
  }
}
}  // namespace uapbridge_pic16
}  // namespace esphome