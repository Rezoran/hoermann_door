#pragma once

#include "../uapbridge_pic16.h" // Including the UAPBridge_pic16 header
#include "esphome/components/output/binary_output.h"
#include "esphome/core/component.h"

namespace esphome {
namespace uapbridge_pic16 {

class UAPBridge_pic16BinaryOutput : public output::BinaryOutput, public Component {
  public:
    void set_uapbridge_pic16_parent(UAPBridge_pic16 *parent) { this->parent_ = parent; }
    void write_state(bool state) override;
    void dump_config() override;
  protected:
    UAPBridge_pic16 *parent_;
};

}  // namespace uapbridge_pic16
}  // namespace esphome
