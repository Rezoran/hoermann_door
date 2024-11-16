#pragma once

#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "../uapbridge_pic16.h"

namespace esphome {
namespace uapbridge_pic16 {

class UAPBridgePic16TextSensor : public text_sensor::TextSensor, public Component
{
  public:
    void set_uapbridge_pic16_parent(UAPBridge_pic16 *parent) { this->parent_ = parent; }
    void setup() override;
    void on_event_triggered();

  private:
    UAPBridge_pic16 *parent_;
    std::string previousState_;
};

}  // namespace uapbridge_pic16
}  // namespace esphome