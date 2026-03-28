#include "Arduino.h"
#include <chrono>
#include <map>

static unsigned long mocked_millis = 0;
static std::map<uint8_t, int> analog_inputs;

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

int analogRead(uint8_t pin) {
    return analog_inputs[pin];
}

void set_analog_input(uint8_t pin, int value) {
    analog_inputs[pin] = value;
}

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

void Adafruit_SSD1306::display() {
    static int frame_count = 0;
    char filename[64];
    snprintf(filename, sizeof(filename), "frame_%s_%04d.json", name.c_str(), frame_count++);
    std::ofstream out(filename);
    out << "{\n  \"width\": " << width << ",\n  \"height\": " << height << ",\n  \"commands\": [\n";
    for (size_t i = 0; i < commands.size(); ++i) {
        const auto& cmd = commands[i];
        out << "    {\"type\": \"text\", \"x\": " << cmd.x << ", \"y\": " << cmd.y
            << ", \"size\": " << (int)cmd.size << ", \"text\": \"" << cmd.text << "\", \"color\": "
            << (cmd.color ? 1 : 0) << "}" << (i == commands.size() - 1 ? "" : ",") << "\n";
    }
    out << "  ]\n}\n";
    std::cout << "Captured frame: " << filename << std::endl;
}

EEPROM_ EEPROM;
LowPower_ LowPower;
