#include "modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome
{
  namespace modbus
  {

    static const char *const TAG = "modbus";

    void Modbus::setup()
    {
      if (this->flow_control_pin_ != nullptr)
      {
        this->flow_control_pin_->setup();
      }
    }
    bool Modbus::is_busy()
    {
      // Check we have out standing requests and are not waiting for response
      for (auto *device : this->devices_)
      {
        // Check if the master is blocked because it is sending out a request
        if (device->address_ == Modbus::MODBUS_MASTER_ID && this->master_is_busy_)
        {
          return true;
        }
        if (this->sniffer_count[device->address_] > 0 && device->front_command_sniffer())
        {
          return true;
        }
      }
      if (this->available() || this->rx_buffer_.size() > 0) //|| (millis() - this->last_modbus_byte_ < 5))
        return true;
      return false;
    }

    void Modbus::loop()
    {
      const uint32_t now = millis();
      // If avaialbe processing will be done in available loop
      if (!this->available() && this->rx_buffer_.size() > 0)
      {
        while (!this->process_modbus_(this->rx_buffer_.size(), now - this->last_modbus_byte_ > MAX_COMMAND_DURATION))
        {
        }
      }

      // stop blocking new send commands after send_wait_time_ ms regardless if a response has been received since then
      if (waiting_for_response != 0 && now - this->last_send_ > send_wait_time_)
      {
        waiting_for_response = 0;
      }

      while (this->available())
      {
        uint8_t byte;
        this->read_byte(&byte);
        this->parse_modbus_byte_(byte);
        this->last_modbus_byte_ = now;
      }
    }

    bool Modbus::parse_modbus_byte_(uint8_t byte)
    {
      size_t at = this->rx_buffer_.size();
      this->rx_buffer_.push_back(byte);
      ESP_LOGV(TAG, "Modbus received Byte  %d (0X%x)", byte, byte);
      return this->process_modbus_(at, false);
    }

    void Modbus::reset()
    {
      // Check for a hanging request
      if (this->role == ModbusRole::SHARED)
      {
        for (auto *device : this->devices_)
        {
          device->clear_command_queue();
          this->sniffer_mode[device->address_] = ModbusMode::UNKOWN;
          this->sniffer_count[device->address_] = 0;
        }
        // Clear ths incomming bus
        uint8_t byte;
        while (this->available())
          this->read_byte(&byte);
        this->rx_buffer_.clear();
        this->waiting_for_response = 0;
        this->master_is_busy_ = false;
        ESP_LOGW(TAG, "Modbus reset sniffer changing to UNKNOWN for all devices");
      }
    }

    void Modbus::reset(uint8_t address)
    {
      // Check for a hanging request
      if (this->role == ModbusRole::SHARED)
      {
        for (auto *device : this->devices_)
        {
          if (device->address_ == address)
          {
            if (this->waiting_for_response == address)
              this->waiting_for_response = 0;
            if (this->sniffer_mode[device->address_] != ModbusMode::UNKOWN || this->sniffer_count[device->address_] != 0)
            {
              ESP_LOGD(TAG, "Modbus reset sniffer changing to UNKNOWN for device address=%d", address);
            }
            this->sniffer_mode[device->address_] = ModbusMode::UNKOWN;
            this->sniffer_count[device->address_] = 0;
            if (address == Modbus::MODBUS_MASTER_ID)
              this->master_is_busy_ = false;

            break;
          }
        }
      }
    }

    bool Modbus::process_modbus_(size_t at, bool retry)
    {
      const uint8_t *raw = &this->rx_buffer_[0];

      // Byte 0: modbus address (match all)
      if (at == 0)
      {
        return true;
      }
      uint8_t address = raw[0];
      uint8_t function_code = raw[1];
      // Byte 2: Size (with modbus rtu function code 4/3)
      // See also https://en.wikipedia.org/wiki/Modbus
      if (at == 2)
      {
        return true;
      }

      uint8_t data_len = raw[2];
      uint8_t data_offset = 3;

      // Per https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf Ch 5 User-Defined function codes
      if (((function_code >= 65) && (function_code <= 72)) || ((function_code >= 100) && (function_code <= 110)))
      {
        // Handle user-defined function, since we don't know how big this ought to be,
        // ideally we should delegate the entire length detection to whatever handler is
        // installed, but wait, there is the CRC, and if we get a hit there is a good
        // chance that this is a complete message ... admittedly there is a small chance is
        // isn't but that is quite small given the purpose of the CRC in the first place

        // Fewer than 2 bytes can't calc CRC
        if (at < 2)
        {
          return true;
        }
        data_len = at - 2;
        data_offset = 1;

        uint16_t computed_crc = crc16(raw, data_offset + data_len);
        uint16_t remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);

        if (computed_crc != remote_crc)
        {
          ESP_LOGW(TAG, "Modbus user-defined function %02X found with failed CRC, raw: %s", function_code, format_hex_pretty(std::vector<uint8_t>(this->rx_buffer_.begin(), this->rx_buffer_.begin() + data_offset + data_len)).c_str());
          this->reset();
          return true;
        }

        ESP_LOGD(TAG, "Modbus user-defined function %02X found", function_code);
      }
      else
      {
        // Check if we have data corruption
        if (this->role == ModbusRole::SHARED && address > MAX_MODBUS_ADDRESS_COUNT)
        {
          ESP_LOGW(TAG, "Modbus address(%d) error ", address);
          this->reset();
          return true;
        }

        ESP_LOGV(TAG, "Modbus data address(%d), raw: %s", address, format_hex_pretty(std::vector<uint8_t>(this->rx_buffer_.begin(), this->rx_buffer_.end())).c_str());

        if (this->role == ModbusRole::SHARED && this->sniffer_count[address] == 0 &&
            (function_code == 0x3 || function_code == 0x4) && this->sniffer_mode[address] != ModbusMode::SERVER)
        {
          this->sniffer_mode[address] = ModbusMode::SERVER;
          data_offset = 2;
          data_len = 4;
          ESP_LOGD(TAG, "Modbus sniffer changing to SERVER for device address=%d and function %02X", address, function_code);
        }

        // Check if we should, change over to server based on CRC
        if (this->role == ModbusRole::SHARED && at >= 8 && this->sniffer_count[address] != 0 &&
            (function_code == 0x3 || function_code == 0x4) &&
            this->sniffer_mode[address] != ModbusMode::SERVER)
        {
          uint16_t computed_crc = crc16(raw, 6);
          uint16_t remote_crc = uint16_t(raw[6]) | (uint16_t(raw[7]) << 8);
          if (computed_crc == remote_crc)
          {
            this->reset(address);
            this->sniffer_mode[address] = ModbusMode::SERVER;
            data_offset = 2;
            data_len = 4;
            ESP_LOGD(TAG, "Modbus reset to SERVER based on crc for address(%d), raw: %s", address, format_hex_pretty(std::vector<uint8_t>(this->rx_buffer_.begin(), this->rx_buffer_.end())).c_str());
          }
        }

        // data starts at 2 and length is 4 for read registers commands
        if ((this->role == ModbusRole::SERVER || this->sniffer_mode[address] == ModbusMode::SERVER) &&
            (function_code == 0x3 || function_code == 0x4))
        {
          data_offset = 2;
          data_len = 4;
        }

        // the response for write command mirrors the requests and data starts at offset 2 instead of 3 for read commands
        if (function_code == 0x5 || function_code == 0x06 || function_code == 0xF || function_code == 0x10)
        {
          data_offset = 2;
          data_len = 4;
        }

        // Error ( msb indicates error )
        // response format:  Byte[0] = device address, Byte[1] function code | 0x80 , Byte[2] exception code, Byte[3-4] crc
        if ((function_code & 0x80) == 0x80)
        {
          data_offset = 2;
          data_len = 1;
        }

        // Byte data_offset..data_offset+data_len-1: Data
        if (at < data_offset + data_len)
        {
          ESP_LOGV(TAG, "AT at (%d) < data_offset (%d) + data_len (%d), buflen(%d)", at, data_offset, data_len, this->rx_buffer_.size());
          return true;
        }

        // Byte 3+data_len: CRC_LO (over all bytes)
        if (at == data_offset + data_len)
        {
          ESP_LOGV(TAG, "at (%d) == data_offset (%d) + data_len (%d), buflen(%d)", at, data_offset, data_len, this->rx_buffer_.size());
          return true;
        }

        // Byte data_offset+len+1: CRC_HI (over all bytes)
        uint16_t computed_crc = crc16(raw, data_offset + data_len);
        uint16_t remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);
        if (computed_crc != remote_crc)
        {
          if (this->disable_crc_)
          {
            ESP_LOGD(TAG, "Modbus (%d) CRC Check failed, but ignored! %02X!=%02X", address, computed_crc, remote_crc);
          }
          else
          {
            ESP_LOGW(TAG, "Modbus (%d) CRC Check failed! %02X!=%02X , raw: %s", address, computed_crc, remote_crc, format_hex_pretty(std::vector<uint8_t>(this->rx_buffer_.begin(), this->rx_buffer_.begin() + data_offset + data_len)).c_str());
            this->reset(); // Will clear all on CRC
            return false;
          }
        }
      }
      std::vector<uint8_t> data(this->rx_buffer_.begin() + data_offset, this->rx_buffer_.begin() + data_offset + data_len);
      bool found = false;
      for (auto *device : this->devices_)
      {
        if (device->address_ == address)
        {
          // Is it an error response?
          if ((function_code & 0x80) == 0x80)
          {
            ESP_LOGD(TAG, "Modbus error function code: 0x%X exception: %d", function_code, raw[2]);
            if (waiting_for_response != 0)
            {
              device->on_modbus_error(function_code & 0x7F, raw[2]);
            }
            else
            {
              // Ignore modbus exception not related to a pending command
              ESP_LOGD(TAG, "Ignoring Modbus error - not expecting a response");
            }
          }

          else if (this->role == ModbusRole::SHARED && device->is_sniffer())
          {
            ESP_LOGV(TAG, "Got Modbus frame from device address=%d, mode %d, raw: %s", address, this->sniffer_mode[address], format_hex_pretty(data).c_str());
            if (this->sniffer_mode[address] == ModbusMode::SERVER && (function_code == 0x3 || function_code == 0x4))
            {
              this->sniffer_count[address] = device->on_modbus_shared_registers(function_code,
                                                                                uint16_t(data[1]) | (uint16_t(data[0]) << 8),
                                                                                uint16_t(data[3]) | (uint16_t(data[2]) << 8));
              if (this->sniffer_count[address] == 0)
              {
                this->reset(); // We did not get a sniffer count indicating a error
                return false;
              }
              else
              {
                this->sniffer_mode[address] = ModbusMode::SNIFFER;
                this->waiting_for_response = address;
                ESP_LOGD(TAG, "Modbus sniffer changing to SNIFFER for device address=%d", address);
              }
            }
            else
            {
              ESP_LOGD(TAG, "Modbus (%d) data %s", address, format_hex_pretty(std::vector<uint8_t>(this->rx_buffer_.begin(), this->rx_buffer_.begin() + data_offset + data_len)).c_str());
              ESP_LOGD(TAG, "Modbus (%d) Processing data %s", address, format_hex_pretty(data).c_str());
              device->on_modbus_data(data);
              if (this->sniffer_count[address] > 0)
                this->sniffer_count[address]--;
              if (this->sniffer_count[address] == 0)
              {
                this->sniffer_mode[address] = ModbusMode::UNKOWN;
                ESP_LOGD(TAG, "Modbus sniffer changing to UNKOWN for device address=%d", address);
              }
            }
          }

          else if (this->role == ModbusRole::SERVER && (function_code == 0x3 || function_code == 0x4))
          {
            device->on_modbus_read_registers(function_code, uint16_t(data[1]) | (uint16_t(data[0]) << 8),
                                             uint16_t(data[3]) | (uint16_t(data[2]) << 8));
          }
          else
          {
            device->on_modbus_data(data);
            if (this->sniffer_count[address] > 0)
              this->sniffer_count[address]--;
            if (this->sniffer_count[address] == 0)
            {
              this->sniffer_mode[address] = ModbusMode::UNKOWN;
              ESP_LOGD(TAG, "Modbus sniffer changing to UNKOWN for device address=%d", address);
            }
          }
          found = true;
          break;
        }
      }

      if (!found)
      {
        this->sniffer_mode[address] = ModbusMode::UNKOWN;
        this->sniffer_count[address] = 0;
        ESP_LOGD(TAG, "Got Modbus frame from unknown device address=%d!,raw: %s", address, format_hex_pretty(data).c_str());

        // ON ECOS the master send, 10 SOC (35155) request, blocking, any capability to send a command to master
        if (this->role == ModbusRole::SHARED && (uint16_t(data[1]) | (uint16_t(data[0]) << 8)) == Modbus::CMD_SOC)
        {
          // Temp work arround
          if (address == Modbus::MAX_MODBUS_ADDRESS_COUNT)
          {
            this->reset();
          }
          this->master_is_busy_ = address != Modbus::MAX_MODBUS_ADDRESS_COUNT;
          ESP_LOGV(TAG, "Changing master busy %d, %d", address, this->master_is_busy_);
        }
      }
      // reduce the buffer with the number or processed bytes = data_offset + data_len + crc(2)
      ESP_LOGD(TAG, "Removing data(%d)", data_offset + data_len + 2);
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + data_offset + data_len + 2);
      waiting_for_response = 0;
      return this->rx_buffer_.size() > 0;
    }

    void Modbus::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Modbus:");
      LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
      ESP_LOGCONFIG(TAG, "  Send Wait Time: %d ms", this->send_wait_time_);
      ESP_LOGCONFIG(TAG, "  CRC Disabled: %s", YESNO(this->disable_crc_));
    }
    float Modbus::get_setup_priority() const
    {
      // After UART bus
      return setup_priority::BUS - 1.0f;
    }

    void Modbus::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
                      uint8_t payload_len, const uint8_t *payload)
    {
      static const size_t MAX_VALUES = 128;

      // Only check max number of registers for standard function codes
      // Some devices use non standard codes like 0x43
      if (number_of_entities > MAX_VALUES && function_code <= 0x10)
      {
        ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
        return;
      }

      std::vector<uint8_t> data;
      data.push_back(address);
      data.push_back(function_code);
      if (this->role == ModbusRole::CLIENT || this->role == ModbusRole::SHARED)
      {
        data.push_back(start_address >> 8);
        data.push_back(start_address >> 0);
        if (function_code != 0x5 && function_code != 0x6)
        {
          data.push_back(number_of_entities >> 8);
          data.push_back(number_of_entities >> 0);
        }
      }

      if (payload != nullptr)
      {
        if (this->role == ModbusRole::SERVER || function_code == 0xF || function_code == 0x10)
        {                              // Write multiple
          data.push_back(payload_len); // Byte count is required for write
        }
        else
        {
          payload_len = 2; // Write single register or coil
        }
        for (int i = 0; i < payload_len; i++)
        {
          data.push_back(payload[i]);
        }
      }

      auto crc = crc16(data.data(), data.size());
      data.push_back(crc >> 0);
      data.push_back(crc >> 8);

      if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(true);

      this->write_array(data);
      this->flush();

      if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(false);
      waiting_for_response = address;
      if (this->role == ModbusRole::SHARED)
      {
        this->sniffer_mode[address] = ModbusMode::CLIENT;
        this->sniffer_count[address]++;
      }
      last_send_ = millis();
      ESP_LOGV(TAG, "Modbus write: %s", format_hex_pretty(data).c_str());
    }

    // Helper function for lambdas
    // Send raw command. Except CRC everything must be contained in payload
    void Modbus::send_raw(const std::vector<uint8_t> &payload)
    {
      if (payload.empty())
      {
        return;
      }

      if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(true);

      auto crc = crc16(payload.data(), payload.size());
      this->write_array(payload);
      this->write_byte(crc & 0xFF);
      this->write_byte((crc >> 8) & 0xFF);
      this->flush();
      if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(false);
      waiting_for_response = payload[0];
      ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
      last_send_ = millis();
    }

  } // namespace modbus
} // namespace esphome
