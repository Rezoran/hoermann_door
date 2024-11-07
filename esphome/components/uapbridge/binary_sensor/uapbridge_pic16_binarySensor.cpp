#include "uapbridge_pic16_binarySensor.h"

namespace esphome {
namespace uapbridge_pic16 {

static const char* const TAG = "uapbridge_pic16.binary_sensor";

// Relay Sensor
void UAPBridge_pic16RelaySensor::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  this->publish_state(this->parent_->get_relay_enabled());
}

void UAPBridge_pic16RelaySensor::on_event_triggered() {
  if (this->parent_->get_relay_enabled() != this->state) {
    this->publish_state(this->parent_->get_relay_enabled());
  }
}

void UAPBridge_pic16RelaySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge_pic16RelaySensor");
}

// Communication Sensor
void UAPBridge_pic16Communication::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  this->publish_state(this->parent_->get_pic16_com());
}

void UAPBridge_pic16Communication::on_event_triggered() {
  if (this->parent_->get_pic16_com() != this->state) {
    this->publish_state(this->parent_->get_pic16_com());
  }
}

void UAPBridge_pic16Communication::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge_pic16Communication");
}

// Error Sensor
void UAPBridge_pic16ErrorSensor::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  this->publish_state(this->parent_->get_error_state());
}

void UAPBridge_pic16ErrorSensor::on_event_triggered() {
  if (this->parent_->get_error_state() != this->state) {
    this->publish_state(this->parent_->get_error_state());
  }
}

void UAPBridge_pic16ErrorSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge_pic16ErrorSensor");
}

void UAPBridge_pic16PrewarnSensor::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  this->publish_state(this->parent_->get_prewarn_state());
}

void UAPBridge_pic16PrewarnSensor::on_event_triggered() {
  if (this->parent_->get_prewarn_state() != this->state) {
    this->publish_state(this->parent_->get_prewarn_state());
  }
}

void UAPBridge_pic16PrewarnSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge_pic16PrewarnSensor");
}

// GotValidBroadcast Sensor (formerly DataHasChanged)
void UAPBridge_pic16GotValidBroadcast::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  this->publish_state(this->parent_->get_valid_broadcast());
}

void UAPBridge_pic16GotValidBroadcast::on_event_triggered() {
  if (this->parent_->get_valid_broadcast() != this->state) {
    this->publish_state(this->parent_->get_valid_broadcast());
  }
}

void UAPBridge_pic16GotValidBroadcast::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge_pic16GotValidBroadcast");
}

}  // namespace uapbridge_pic16
}  // namespace esphome
