#pragma once

#include "esphome/core/gpio.h"


namespace esphome
{
    namespace max14830
    {
        class MAX14830;
        class MAX14830GPIOPin : public GPIOPin
        {
            constexpr static const char *const TAG = "max14830_gpio_pin";
        public:

            void setup() override;
            void pin_mode(gpio::Flags flags) override;
            bool digital_read() override;
            void digital_write(bool value) override;
            std::string dump_summary() const override;

            void set_parent(MAX14830 *parent) { parent_ = parent; }
            void set_pin(uint8_t pin) { pin_ = pin; }
            void set_inverted(bool inverted) { inverted_ = inverted; }
            void set_flags(gpio::Flags flags) { flags_ = flags; }

            gpio::Flags get_flags() const override { return this->flags_; }

        protected:
            MAX14830 *parent_;
            uint8_t pin_;
            bool inverted_;
            gpio::Flags flags_;
        };

    } // namespace max14830
} // namespace esphome
