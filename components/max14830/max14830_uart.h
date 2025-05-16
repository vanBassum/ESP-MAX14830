#pragma once
#include "esphome/components/uart/uart.h"

namespace esphome
{
    namespace max14830
    {
        class MAX14830;

        enum FlowControl
        {
            UART_CFG_FLOW_CTRL_NONE     = 0x00,
            UART_CFG_FLOW_CTRL_RTS_CTS  = 0x01,
            UART_CFG_FLOW_CTRL_DTR_DSR  = 0x02,
            UART_CFG_FLOW_CTRL_RS485    = 0x03,
        };

        class MAX14830UART : public uart::UARTComponent
        {

        public:
            void write_array(const uint8_t *data, size_t len) override;
            bool peek_byte(uint8_t *data) override;
            bool read_array(uint8_t *data, size_t len) override;
            int available() override;
            void flush() override;

            void set_parent(MAX14830 *parent);
            void set_port(uint8_t port);
            void set_baud_rate(uint32_t baud_rate);
            void set_stop_bits(uint8_t stop_bits);
            void set_data_bits(uint8_t data_bits);
            void set_parity(esphome::uart::UARTParityOptions parity);
            void set_flow_control(FlowControl flow_control);

        protected:
            ///void check_logger_conflict() = 0;

        private:
            MAX14830 *parent_{nullptr};
            uint8_t port_{-1};
            uint32_t baud_rate_{9600};
            uint8_t stop_bits_{1};
            uint8_t data_bits_{8};
            esphome::uart::UARTParityOptions parity_{esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE};
            FlowControl flow_control_{UART_CFG_FLOW_CTRL_NONE};

            void UpdateConfig();
        };

    } // namespace max14830
} // namespace esphome
