#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "../uapbridge_pic16.h"

namespace esphome {
namespace uapbridge_pic16 {

class UAPBridge_pic16RelaySensor : public binary_sensor::BinarySensor, public Component {
public:
  void set_uapbridge_pic16_parent(UAPBridge_pic16* parent) {
    this->parent_ = parent;
  }
  void setup() override;
  void on_event_triggered();
  void dump_config() override;

protected:
  UAPBridge_pic16* parent_;
};

class UAPBridge_pic16Communication : public binary_sensor::BinarySensor, public Component {
public:
  void set_uapbridge_pic16_parent(UAPBridge_pic16* parent) {
    this->parent_ = parent;
  }
  void setup() override;
  void on_event_triggered();
  void dump_config() override;

protected:
  UAPBridge_pic16* parent_;
};

class UAPBridge_pic16ErrorSensor : public binary_sensor::BinarySensor, public Component {
public:
  void set_uapbridge_pic16_parent(UAPBridge_pic16* parent) {
    this->parent_ = parent;
  }
  void setup() override;
  void on_event_triggered();
  void dump_config() override;

protected:
  UAPBridge_pic16* parent_;
};

class UAPBridge_pic16PrewarnSensor : public binary_sensor::BinarySensor, public Component {
public:
  void set_uapbridge_pic16_parent(UAPBridge_pic16* parent) {
    this->parent_ = parent;
  }
  void setup() override;
  void on_event_triggered();
  void dump_config() override;

protected:
  UAPBridge_pic16* parent_;
};

class UAPBridge_pic16GotValidBroadcast : public binary_sensor::BinarySensor, public Component {
public:
  void set_uapbridge_pic16_parent(UAPBridge_pic16* parent) {
    this->parent_ = parent;
  }
  void setup() override;
  void on_event_triggered();
  void dump_config() override;

protected:
  UAPBridge_pic16* parent_;
};

}  // namespace uapbridge_pic16
}  // namespace esphome
