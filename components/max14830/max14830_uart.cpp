#include "max14830_uart.h"
#include "max14830.h"

namespace esphome
{
    namespace max14830
    {

        void write_array(const uint8_t *data, size_t len) {
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return;
            }

            parent_->UartWrite(port_, data, len);
        }

        bool peek_byte(uint8_t *data) {
            ESP_LOGE(TAG, "peek_byte not implemented");
        }

        bool read_array(uint8_t *data, size_t len){
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return;
            }
            parent_->UartRead(port_, data, len);
        }

        int available() {
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return;
            }

            parent_->UartAvailable(port_);
        }

        void flush() {
            
        }

        void MAX14830UART::set_parent(MAX14830 *parent)
        {
            this->parent_ = parent;
            UpdateConfig();
        }
        void MAX14830UART::set_port(uint8_t port)
        {
            this->port_ = port;
            UpdateConfig();
        }
        void MAX14830UART::set_baud_rate(uint32_t baud_rate)
        {
            this->baud_rate_ = baud_rate;
            UpdateConfig();
        }

        void MAX14830UART::set_stop_bits(uint8_t stop_bits)
        {
            this->stop_bits_ = stop_bits;
            UpdateConfig();
        }

        void MAX14830UART::set_data_bits(uint8_t data_bits)
        {
            this->data_bits_ = data_bits;
            UpdateConfig();
        }
        
        void MAX14830UART::set_parity(esphome::uart::UARTParityOptions parity)
        {
            this->parity_ = parity;
            UpdateConfig();
        }

        void MAX14830UART::set_flow_control(FlowControl flow_control)
        {
            this->flow_control_ = flow_control;
            UpdateConfig();
        }

        void MAX14830UART::UpdateConfig()
        {   
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return;
            }

            if (port_ == -1)
            {
                ESP_LOGE(TAG, "Port not set");
                return;
            }

            parent_->UartConfigure(port_, baud_rate_, parity_, stop_bits_, data_bits_, flow_control_);
        }

    } // namespace max14830
} // namespace esphome
