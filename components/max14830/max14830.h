#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"

// This should become config TODO
#define MAX14830_USE_XTAL
#define MAX14830_CLK 4000000
#define MAX14830_BRGCFG_CLKDIS_BIT (1 << 6) /* Clock Disable */
#define MAX14830_REV_ID (0xb0)
#define MAX14830_FIFO_MAX 128

namespace esphome
{
    namespace max14830
    {

        class MAX14830 : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_8MHZ>
        {
        public:
            void setup() override;
            void dump_config() override;
            float get_setup_priority() const override;

            bool digital_read(uint8_t pin);
            void digital_write(uint8_t pin, bool value);
            void pin_mode(uint8_t pin, gpio::Flags flags);


        private:
            bool Detect();
            bool SetRefClock(uint32_t *clk);
            bool portInit(uint8_t port);

            bool regmap_write(uint8_t cmd, uint8_t value);
            uint8_t regmap_read(uint8_t cmd);
            uint8_t max310x_port_read(uint8_t port, uint8_t cmd);
            bool max310x_port_write(uint8_t port, uint8_t cmd, uint8_t value);
            bool max310x_port_update(uint8_t port, uint8_t cmd, uint8_t mask, uint8_t value);
            uint8_t max310x_update_best_err(uint64_t f, int64_t *besterr);
            bool Max14830_WriteBufferPolled(uint8_t cmd, const uint8_t *cmdData, uint8_t count);
            bool Max14830_ReadBufferPolled(uint8_t cmd, uint8_t *cmdData, uint8_t *replyData, uint8_t count);


            uint8_t gpioConfBuffer[4] = {};
            uint16_t gpioDataBuffer = 0;

        };

    } // namespace max14830
} // namespace esphome
