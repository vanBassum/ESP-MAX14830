#pragma once

#include "esphome/core/gpio.h"

namespace esphome
{
    namespace max14830
    {

        class MAX14830;

        class MAX14830GPIOPin : public GPIOPin
        {
        public:
            void set_parent(MAX14830 *parent);
            void set_pin(uint8_t pin);

            void setup() override;
            void pin_mode(gpio::Flags flags) override;
            bool digital_read() override;
            void digital_write(bool value) override;

        protected:
            MAX14830 *parent_;
            uint8_t pin_;
        };

    } // namespace max14830
} // namespace esphome
