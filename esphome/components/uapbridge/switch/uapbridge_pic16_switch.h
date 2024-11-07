#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../uapbridge_pic16.h"

namespace esphome {
namespace uapbridge_pic16 {

    class UAPBridge_pic16SwitchVent : public switch_::Switch, public Component
    {
      public:
        void set_uapbridge_pic16_parent(UAPBridge_pic16 *parent) { this->parent_ = parent; }
        void setup() override;
        void on_event_triggered();
        void write_state(bool state) override;
        void dump_config() override;
      private:
        UAPBridge_pic16 *parent_;
        bool previousState_ = false;
    };
    class UAPBridge_pic16SwitchLight : public switch_::Switch, public Component
    {
      public:
        void set_uapbridge_pic16_parent(UAPBridge_pic16 *parent) { this->parent_ = parent; }
        void setup() override;
        void on_event_triggered();
        void write_state(bool state) override;
        void dump_config() override;
      private:
        UAPBridge_pic16 *parent_;
        bool previousState_ = false;
    };
  }
}