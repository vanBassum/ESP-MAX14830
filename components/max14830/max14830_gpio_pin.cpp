#include "max14830_gpio_pin.h"
#include "max14830.h"

namespace esphome {
namespace max14830 {

void MAX14830GPIOPin::set_parent(MAX14830 *parent) {
  this->parent_ = parent;
}

void MAX14830GPIOPin::set_pin(uint8_t pin) {
  this->pin_ = pin;
}

void MAX14830GPIOPin::setup() {
  this->parent_->setup_pin(this->pin_, gpio::FLAG_OUTPUT);
}

void MAX14830GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->setup_pin(this->pin_, flags);
}

bool MAX14830GPIOPin::digital_read() {
  return this->parent_->read_pin(this->pin_);
}

void MAX14830GPIOPin::digital_write(bool value) {
  this->parent_->write_pin(this->pin_, value);
}

}  // namespace max14830
}  // namespace esphome
