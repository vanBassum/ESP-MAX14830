#include "max14830_uart.h"
#include "max14830.h"

namespace esphome
{
    namespace max14830
    {

        void MAX14830UART::write_array(const uint8_t *data, size_t len)
        {
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return;
            }

            parent_->UartWrite(port_, data, len);
        }

        bool MAX14830UART::peek_byte(uint8_t *data)
        {
            ESP_LOGE(TAG, "peek_byte not implemented");
            return false;
        }

        bool MAX14830UART::read_array(uint8_t *data, size_t len)
        {
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return false;
            }
            return parent_->UartRead(port_, data, len) == len;
        }

        int MAX14830UART::available()
        {
            if (parent_ == nullptr)
            {
                ESP_LOGE(TAG, "Parent not set");
                return 0;
            }

            return parent_->UartAvailable(port_);
        }

        void MAX14830UART::flush()
        {
        }

        void esphome::max14830::MAX14830UART::setup()
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

        void esphome::max14830::MAX14830UART::dump_config()
        {
            ESP_LOGCONFIG("max14830.uart", "MAX14830 UART port %d", port_);
        }

        void MAX14830UART::set_parent(MAX14830 *parent)
        {
            this->parent_ = parent;
        }
        void MAX14830UART::set_port(uint8_t port)
        {
            this->port_ = port;
        }
        void MAX14830UART::set_baud_rate(uint32_t baud_rate)
        {
            this->baud_rate_ = baud_rate;
        }

        void MAX14830UART::set_stop_bits(uint8_t stop_bits)
        {
            this->stop_bits_ = stop_bits;
        }

        void MAX14830UART::set_data_bits(uint8_t data_bits)
        {
            this->data_bits_ = data_bits;
        }

        void MAX14830UART::set_parity(esphome::uart::UARTParityOptions parity)
        {
            this->parity_ = parity;
        }

        void MAX14830UART::set_flow_control(FlowControl flow_control)
        {
            this->flow_control_ = flow_control;
        }

        void MAX14830UART::check_logger_conflict()
        {
        }

    } // namespace max14830
} // namespace esphome
