#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/output/binary_output.h"
#include "../uapbridge_pic16.h"

namespace esphome {
namespace uapbridge_pic16 {

class UAPBridge_pic16Light : public light::LightOutput, public Component {
  public:
    void set_uapbridge_pic16_parent(UAPBridge_pic16 *parent) { this->parent_ = parent; }
    light::LightTraits get_traits() override;
    void set_output(output::BinaryOutput *output) { output_ = output; }
    void write_state(light::LightState *state) override;
    void setup() override;
    void on_event_triggered();
    void dump_config() override;
    void setup_state(light::LightState *state) { state_ = state; }

  protected:
    UAPBridge_pic16 *parent_;
    output::BinaryOutput *output_;
    light::LightState *state_{nullptr};
};

}  // namespace uapbridge_pic16
}  // namespace esphome
