#pragma once
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/gpio.h"

namespace esphome
{
    namespace max14830
    {

        class MAX14830
        {
        public:
            void setup();
            void dump_config();
            void set_device_address(uint8_t address);

            void setup_pin(uint8_t pin, gpio::Flags flags);
            void write_pin(uint8_t pin, bool value);
            bool read_pin(uint8_t pin);

        protected:
            uint8_t device_address_;
        };

    }
}