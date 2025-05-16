#include "max14830_gpio_pin.h"
#include "max14830.h"

namespace esphome
{
    namespace max14830
    {

        void MAX14830GPIOPin::setup()
        {
            pin_mode(flags_);
        }

        void MAX14830GPIOPin::pin_mode(gpio::Flags flags)
        {
            this->parent_->pin_mode(this->pin_, flags);
        }

        bool MAX14830GPIOPin::digital_read()
        {
            return this->parent_->digital_read(this->pin_) != this->inverted_;
        }

        void MAX14830GPIOPin::digital_write(bool value)
        {
            this->parent_->digital_write(this->pin_, value != this->inverted_);
        }

        std::string MAX14830GPIOPin::dump_summary() const
        {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%u via PCF8574", pin_);
            return buffer;
        }

    } // namespace max14830
} // namespace esphome
