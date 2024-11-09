#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#define CYCLE_TIME                1   // ms
#define CYCLE_TIME_SLOW         100   // ms

#define BROADCAST_ADDR            0x00
#define UAP1_ADDR                 0x28
#define UAP1_ADDR_MASTER          128

#define UAP1_TYPE                 0x14

#define CMD_SLAVE_SCAN            0x01
#define CMD_SLAVE_STATUS_REQUEST  0x20
#define CMD_SLAVE_STATUS_RESPONSE 0x29

namespace esphome {
namespace uapbridge {

class UAPBridge : public uart::UARTDevice, public Component {
  public:
    // Enumeration for actions
    enum hoermann_action_t {
      hoermann_action_stop = 0x0000,
      hoermann_action_open = 0x1001,
      hoermann_action_close = 0x1002,
      hoermann_action_venting = 0x1010,
      hoermann_action_toggle_light = 0x1008,
      hoermann_action_none = 0x1000
    };

    // Enumeration for states
    enum hoermann_state_t {
      hoermann_state_open         = 0x0001,
      hoermann_state_closed       = 0x0002,
      hoermann_state_opt_relay    = 0x0004,
      hoermann_state_light_relay  = 0x0008,
      hoermann_state_error        = 0x0010,
      hoermann_state_direction    = 0x0020,
      hoermann_state_moving       = 0x0040,
      hoermann_state_opening      = 0x0040,
      hoermann_state_closing      = 0x0060,
      hoermann_state_ventpos      = 0x0080,
      hoermann_state_prewarn      = 0x0100
    };

    void setup() override;
    void loop() override;
    void dump_config() override;
    void add_on_state_callback(std::function<void()> &&callback);
    void set_rts_pin(InternalGPIOPin *rts_pin) { this->rts_pin_ = rts_pin; }
    void set_auto_correction(bool value) { this->auto_correction = value; }

    void action_open();
    void action_close();
    void action_stop();
    void action_venting();
    void action_toggle_light();

    hoermann_state_t get_state();
    std::string get_state_string();
    void set_venting(bool state);
    bool get_venting_enabled();
    void set_light(bool state);
    bool get_light_enabled();
    bool get_relay_enabled() const { return this->relay_enabled; }
    void set_relay_enabled(bool value) { this->relay_enabled = value; }
    bool get_error_state() const { return this->error_state; }
    void set_error_state(bool value) { this->error_state = value; }
    bool get_prewarn_state() const { return this->prewarn_state; }
    void set_prewarn_state(bool value) { this->prewarn_state = value; }
    bool get_valid_broadcast() const { return this->valid_broadcast; }
    void set_valid_broadcast(bool value) { this->valid_broadcast = value; }
    bool has_data_changed();
    void clear_data_changed_flag();

  protected:
    // yaml parameters
    InternalGPIOPin *rts_pin_;
    bool auto_correction = false;
    // \yaml parameters
    CallbackManager<void()> state_callback_;
    hoermann_state_t state = nullptr;
    hoermann_state_t last_door_state = nullptr;
    hoermann_action_t next_action = hoermann_action_none;
    std::string state_string = "unknown";

    bool venting_enabled = false;
    bool light_enabled = false;
    bool relay_enabled = false;
    bool error_state = false;
    bool prewarn_state = false;
    bool pic16_com = false;
    bool valid_broadcast = false;
    bool data_has_changed = false;

    uint32_t last_parse_time = millis();

    // Internal methods
    void handle_state_change(hoermann_state_t new_state);
    char* printData(uint8_t *p_data, uint8_t from, uint8_t to);
    void update_boolean_state(const char * name, bool &current_state, bool new_state);
    uint32_t lastCall       = 0;
    uint32_t lastCallSlow   = 0;
    uint16_t broadcast_status = 0;
    uint16_t broadcast_status_old = 0;
    bool ignoreNextEvent = false;     // will also ignore wrong edge detection after reset
    bool autoErrorCorrInProgress = false;
    uint8_t    rxData[5]       = {0, 0, 0, 0, 0};
    uint8_t    txData[6]       = {0, 0, 0, 0, 0, 0};
    uint8_t    txLength        = 0;
    uint8_t    byteCnt         = 0;
    uint32_t   sendTime        = 0;
    void loop_fast();
    void loop_slow();
    void receive();
    void transmit();
    uint8_t UAPBridge::calc_crc8(uint8_t *p_data, uint8_t length);
    static const uint8_t crctable[256] = {
      0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
      0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
      0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
      0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
      0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
      0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
      0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
      0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
      0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
      0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
      0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
      0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
      0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
      0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
      0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
      0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
    };
};

}  // namespace uapbridge
}  // namespace esphome