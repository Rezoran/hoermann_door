#include "uapbridge.h"

namespace esphome {
namespace uapbridge {
static const char *const TAG = "uapbridge";

void UAPBridge::setup() {
  ESP_LOGCONFIG(TAG, "Garage setup called!");
}

void UAPBridge::loop() {
  this->loop_fast();
  this->loop_slow();
}
void UAPBridge::loop_fast() {
  this->receive();

  if (millis() - this->lastCall < CYCLE_TIME) {
    // avoid unnecessary frequent calls 
    return;
  }
  this->lastCall = millis();
  if(this->sendTime!=0 && (millis() >= this->sendTime)) {
    ESP_LOGD(TAG, "loop: transmitting");
    this->transmit();
    this->sendTime = 0;
  }
}
/**
 * this seems to be a function that only logs stati and makes errorcorrections
 */
void UAPBridge::loop_slow() {
  if (millis() - this->lastCallSlow < CYCLE_TIME_SLOW) {
    // avoid unnecessary frequent calls 
    return;
  }
  this->lastCallSlow = millis();

    if (this->ignoreNextEvent) {
      this->ignoreNextEvent = false;
    } else {
      hoermann_state_t new_state = hoermann_state_stopped;

      if ((broadcast_status & hoermann_state_open) == hoermann_state_open) {
        new_state = hoermann_state_open;
      } else if ((broadcast_status & hoermann_state_closed) == hoermann_state_closed) {
        new_state = hoermann_state_closed;
      } else if ((broadcast_status & (hoermann_state_direction | hoermann_state_moving)) == hoermann_state_opening) {
        new_state = hoermann_state_opening;
      } else if ((broadcast_status & (hoermann_state_direction | hoermann_state_moving)) == hoermann_state_closing) {
        new_state = hoermann_state_closing;
      } else if ((broadcast_status & hoermann_state_ventpos) == hoermann_state_ventpos) {
        new_state = hoermann_state_ventpos;
      } else if ((broadcast_status & hoermann_state_error) == hoermann_state_error) {
        new_state = hoermann_state_error;
      }

      if (new_state != this->state) {
        this->handle_state_change(new_state);
      }

      this->update_boolean_state("relay", this->relay_enabled, (this->broadcast_status & hoermann_state_opt_relay));
      this->update_boolean_state("light", this->light_enabled, (this->broadcast_status & hoermann_state_light_relay));
      this->update_boolean_state("vent", this->venting_enabled, (this->broadcast_status & hoermann_state_ventpos));
      this->update_boolean_state("err", this->error_state, (this->broadcast_status & hoermann_state_error));
      this->update_boolean_state("prewarn", this->prewarn_state, (this->broadcast_status & hoermann_state_prewarn));

      // --- Auto Error Correction ---
      if(/*cfgAutoErrorCorr*/true) {
        // if error just came up
        if (((broadcast_status & hoermann_state_error) == hoermann_state_error) && ((broadcast_status_old & hoermann_state_error) != hoermann_state_error)) {
          // if an error is detected and door is open/closed then try to reset it by requesting opening/closing without movement
          ESP_LOGD(TAG, "autocorrection started");
          if (broadcast_status & UAP_STATUS_OPEN) {
            this->setCommand(true, &HoermannCommandUAP::STARTOPENDOOR);
          } else if (broadcast_status & UAP_STATUS_CLOSED) {
            this->setCommand(true, &HoermannCommandUAP::STARTCLOSEDOOR);
          }
          autoErrorCorrInProgress = true;
        }
        // HINT: i guess if light is on it is sufficent to toggle the light off to reset the error
        // HINT: propably not. this will disable the lamp after correcting the error
        // or both
        if (autoErrorCorrInProgress && (broadcast_status & UAP_STATUS_LIGHT_RELAY)) {
          this->setCommand(true, &HoermannCommandUAP::STARTTOGGLELAMP);
          autoErrorCorrInProgress = false;
        }
      }
      // --- Auto Error Correction ---
    }
}

void UAPBridge::receive() {
  uint8_t   length  = 0;
  uint8_t   counter = 0;
  bool   newData = false;
  while (this->available() > 0) {
#if ESPHOME_LOGLEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE
    if (this->byteCnt > 5) {
    	// data have not been fetched and will be ignored --> log them at least for debugging purposes
    	char temp[4];
    	sprintf(temp, "%02X ", this->rxData[0]);
    	ESP_LOGVV(TAG, "in receive while avaiable: %s", temp);
    }
#endif
    // shift old elements and read new; only the last 5 bytes are evaluated; if there are more in the buffer, the older ones are ignored
    for (uint8_t i = 0; i < 4; i++) {
      rxData[i] = rxData[i+1];
    }
    if(this->read_byte(&rxData[4])){
      //if read was successful
      byteCnt++;
    }
    newData = true;
  }
  if (newData) {
    ESP_LOGVV(TAG, "new data received");
    newData = false;
    // Slave scan
    // 28 82 01 80 06
    if (rxData[0] == UAP1_ADDR) {
      length = rxData[1] & 0x0F;
      if (rxData[2] == CMD_SLAVE_SCAN && rxData[3] == UAP1_ADDR_MASTER && length == 2 && calc_crc8(rxData, length + 3) == 0x00) {
        ESP_LOGVV(TAG, "SlaveScan: %s", printData(this->rxData, 0, 5));
        ESP_LOGV(TAG, "->      SlaveScan"); 
        counter = (rxData[1] & 0xF0) + 0x10;
        txData[0] = UAP1_ADDR_MASTER;
        txData[1] = 0x02 | counter;
        txData[2] = UAP1_TYPE;
        txData[3] = UAP1_ADDR;
        txData[4] = calc_crc8(txData, 4);
        txLength = 5;
        sendTime = millis() + TX_DELAY;
      }
    }
    // Broadcast status
    // 00 92 12 02 35
    if (rxData[0] == BROADCAST_ADDR) {
      length = rxData[1] & 0x0F;
      if (length == 2 && calc_crc8(rxData, length + 3) == 0x00) {
        ESP_LOGVV(TAG, "Broadcast: %s", printData(this->rxData, 0, 5));
        ESP_LOGV(TAG, "->      Broadcast"); 
        broadcast_status = rxData[2];
        broadcast_status |= (uint16_t)rxData[3] << 8;
      }
    }
    // Slave status request (only 4 byte --> other indices of rxData!)
    // 28 A1 20 2E
    if (rxData[1] == UAP1_ADDR) {
      length = rxData[2] & 0x0F;
      if (rxData[3] == CMD_SLAVE_STATUS_REQUEST && length == 1 && calc_crc8(&rxData[1], length + 3) == 0x00) {
        ESP_LOGVV(TAG, "Slave status request: %s", printData(this->rxData, 1, 5));
        ESP_LOGV(TAG, "->      Slave status request");
        counter = (rxData[2] & 0xF0) + 0x10;
        txData[0] = UAP1_ADDR_MASTER;
        txData[1] = 0x03 | counter;
        txData[2] = CMD_SLAVE_STATUS_RESPONSE;
        txData[3] = (uint8_t)(nextCommand & 0xFF);
        txData[4] = (uint8_t)((nextCommand >> 8) & 0xFF);
        nextCommand = hoermann_action_none;
        txData[5] = calc_crc8(txData, 5);
        txLength = 6;
        sendTime = millis() + TX_DELAY;
      }
    }
    // Update valid_broadcast if a non-default message is received
    if (!this->valid_broadcast && (this->rxData[3] != 0 || this->rxData[4] != 0)) {
      this->valid_broadcast = true;
      this->data_has_changed = true;
    }
#if ESPHOME_LOGLEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE
    // just print the data
    if (this->byteCnt >= 5) {
      ESP_LOGVV(TAG, "Just printed: %s", printData(this->rxData, 0, 5));
    }	
#endif
  }
}

void UAPBridge::transmit() {
  ESP_LOGVV(TAG, "Transmit: %s", printData(this->txData, 0, this->txLength));
  // Generate Sync break
  if (this->rts != -1) {
    digitalWrite(this->rts, HIGH);		// LOW = listen, HIGH = transmit
  }
  this->set_baud_rate(9600);
  this->set_data_bits(7);
  this->set_parity(esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE);
  this->set_stop_bits(1);
  this->write_byte(0x00);
  this->flush();

  // Transmit
  this->set_baud_rate(19200);
  this->set_data_bits(8);
  this->set_parity(esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE);
  this->set_stop_bits(1);
  this->write_array(txData, txLength);
  this->flush();

  
  ESP_LOGVV(TAG, "TX duration: %dms", millis() - this->sendTime);
}
/**
 * Helper to set next Command and *not* skip Current Command before end was sent
 */
void UAPBridge::setCommand(bool cond, const hoermann_action_t *command) {
  if (cond)
  {
    if (nextCommand != hoermann_action_none)
    {
      ESP_LOGW(TAG, "Last Command was not yet fetched by modbus!");
    }
    else
    {
      this->next_action = command;
      this->ignoreNextEvent = true;
    }
  }
}

void UAPBridge::add_on_state_callback(std::function<void()> &&callback) {
  this->state_callback_.add(std::move(callback));
}

void UAPBridge::action_open() {
  ESP_LOGD(TAG, "Action: open called");
  this->setCommand(true, hoermann_action_open);
}

void UAPBridge::action_close() {
  ESP_LOGD(TAG, "Action: close called");
  this->setCommand(true, hoermann_action_close);
}

void UAPBridge::action_stop() {
  ESP_LOGD(TAG, "Action: stop called");
  this->setCommand(true, hoermann_action_stop);
}

void UAPBridge::action_venting() {
  ESP_LOGD(TAG, "Action: venting called");
  this->setCommand(true, hoermann_action_venting);
}

void UAPBridge::action_toggle_light() {
  ESP_LOGD(TAG, "Action: toggle light called");
  this->setCommand(true, hoermann_action_toggle_light);
}

UAPBridge::hoermann_state_t UAPBridge::get_state() {
  return this->state;
}

std::string UAPBridge::get_state_string() {
  return this->state_string;
}

void UAPBridge::set_venting(bool state) {
  if (state) {
    this->action_venting();
  } else {
    this->action_close();
  }
  ESP_LOGD(TAG, "Venting state set to %s", state ? "ON" : "OFF");
}

bool UAPBridge::get_venting_enabled() {
  return this->venting_enabled;
}

void UAPBridge::set_light(bool state) {
  this->setCommand(state, hoermann_action_toggle_light);
  ESP_LOGD(TAG, "Light state set to %s", state ? "ON" : "OFF");
}

bool UAPBridge::get_light_enabled() {
  return this->light_enabled;
}

bool UAPBridge::has_data_changed() {
  return this->data_has_changed;
}

void UAPBridge::clear_data_changed_flag() {
  this->data_has_changed = false;
}

uint8_t UAPBridge::calc_crc8(uint8_t *p_data, uint8_t length) {
  uint8_t i;
  uint8_t data;
  uint8_t crc = 0xF3;
  
  for(i = 0; i < length; i++) {
    /* XOR-in next input byte */
    data = *p_data ^ crc;
    p_data++;
    /* get current CRC value = remainder */
    crc = crctable[data];
  }
  
  return crc;
}

char* UAPBridge::printData(uint8_t *p_data, uint8_t from, uint8_t to) {
	char temp[4];
	static char output[30];

	sprintf(output, "%5u: ", millis() & 0xFFFFu);
	for (uint8_t i = from; i < to; i++) {
		sprintf(temp, "%02X ", p_data[i]);
		strcat(output, temp);
	}
	this->byteCnt = 0;
  return &output[0];
}

void UAPBridge::handle_state_change(hoermann_state_t new_state) {
  this->state = new_state;

  switch (new_state) {
    case hoermann_state_open:
      this->state_string = "Open";
      break;
    case hoermann_state_closed:
      this->state_string = "Closed";
      break;
    case hoermann_state_opening:
      this->state_string = "Opening";
      break;
    case hoermann_state_closing:
      this->state_string = "Closing";
      break;
    case hoermann_state_ventpos:
      this->state_string = "Venting";
      break;
    case hoermann_state_error:
      this->state_string = "Error";
      break;
    case hoermann_state_stopped:
    default:
      this->state_string = "Stopped";
      break;
  }

  this->data_has_changed = true;
}

void UAPBridge::update_boolean_state(const char * name, bool &current_state, bool new_state) {
  ESP_LOGV(TAG, "update_boolean_state: %s from %s to %s", name, current_state ? "true" : "false", new_state ? "true" : "false");
  if (current_state != new_state) {
    current_state = new_state;
    this->data_has_changed = true;
  }
}

}  // namespace uapbridge
}  // namespace esphome
