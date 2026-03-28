#include "Arduino.h"
#include <chrono>

static unsigned long mocked_millis = 0;

unsigned long millis() {
    return mocked_millis;
}

void set_millis(unsigned long m) {
    mocked_millis = m;
}

void delay(unsigned long ms) {
    mocked_millis += ms;
}

void pinMode(uint8_t pin, uint8_t mode) {}
int digitalRead(uint8_t pin) { return 0; }
void digitalWrite(uint8_t pin, uint8_t val) {}
int analogRead(uint8_t pin) { return 512; }

static void (*interrupt_handlers[20])() = {nullptr};
void attachInterrupt(uint8_t pin, void (*handler)(), int mode) {
    if (pin < 20) interrupt_handlers[pin] = handler;
}

void trigger_interrupt(uint8_t pin) {
    if (pin < 20 && interrupt_handlers[pin]) interrupt_handlers[pin]();
}

uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }

void noInterrupts() {}
void interrupts() {}

Serial_ Serial;
void Serial_::begin(unsigned long baud) {}
void Serial_::print(const char* s) { std::cout << s; }
void Serial_::print(float f, int p) { std::cout << f; }
void Serial_::print(int i) { std::cout << i; }
void Serial_::println(const char* s) { std::cout << s << std::endl; }

EEPROM_ EEPROM;

LowPower_ LowPower;
