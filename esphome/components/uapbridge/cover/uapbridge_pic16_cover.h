#pragma once
#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "../uapbridge_pic16.h"

namespace esphome {
namespace uapbridge_pic16 {

class UAPBridge_pic16Cover : public cover::Cover, public Component {
 public:
  void set_uapbridge_pic16_parent(UAPBridge_pic16 *parent) { this->parent_ = parent; }
  void setup() override;
  void on_event_triggered();
  void control(const cover::CoverCall &call) override;
  cover::CoverTraits get_traits() override;
 protected:
    UAPBridge_pic16 *parent_;
    cover::CoverOperation previousOperation_ = cover::COVER_OPERATION_IDLE;
    UAPBridge_pic16::hoermann_state_t previousState_ = UAPBridge_pic16::hoermann_state_t::hoermann_state_unkown;
};

}  // namespace uapbridge_pic16
}  // namespace esphome

