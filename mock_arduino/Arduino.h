#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <chrono>
#include <stdint.h>
#include <string>

#define PI 3.14159265358979323846
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define CHANGE 1
#define FALLING 2
#define RISING 3
#define WHITE 1
#define SSD1306_SWITCHCAPVCC 0x2

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

typedef uint8_t byte;

unsigned long millis();
void set_millis(unsigned long m);
void delay(unsigned long);
void pinMode(uint8_t, uint8_t);
int digitalRead(uint8_t);
void digitalWrite(uint8_t, uint8_t);
int analogRead(uint8_t);
void set_analog_input(uint8_t pin, int value);
void attachInterrupt(uint8_t, void (*)(), int);
uint8_t digitalPinToInterrupt(uint8_t);
void trigger_interrupt(uint8_t pin);
void noInterrupts();
void interrupts();

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
using std::isnan;

class Serial_ {
public:
    void begin(unsigned long);
    void print(const char*);
    void print(float, int);
    void print(int);
    void println(const char*);
};
extern Serial_ Serial;

struct DrawCmd {
    std::string type;
    int16_t x, y;
    uint8_t size;
    std::string text;
    bool color;
};

class Adafruit_SSD1306 {
    int width, height;
    int16_t cur_x = 0, cur_y = 0;
    uint8_t cur_size = 1;
    bool cur_color = WHITE;
    std::vector<DrawCmd> commands;
    std::string name = "main";

public:
    Adafruit_SSD1306(int w, int h, int mosi, int clk, int dc, int reset, int cs) : width(w), height(h) {}
    Adafruit_SSD1306(int w, int h) : width(w), height(h) {}
    Adafruit_SSD1306(int w, int h, int mosi, int clk, int dc, int rst) : width(w), height(h) {}

    void set_name(const std::string& n) { name = n; }
    void begin(uint8_t) {}
    void clearDisplay() { commands.clear(); }
    void setTextSize(uint8_t s) { cur_size = s; }
    void setTextColor(uint8_t c) { cur_color = (c == WHITE); }
    void setCursor(int16_t x, int16_t y) { cur_x = x; cur_y = y; }

    void print(const std::string& s) {
        commands.push_back({"text", cur_x, cur_y, cur_size, s, cur_color});
        cur_x += s.length() * 6 * cur_size;
    }
    void print(const char* s) { print(std::string(s)); }
    void print(float f, int p = 2) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", p, f);
        print(std::string(buf));
    }
    void print(int i) { print(std::to_string(i)); }
    void println(const char* s) { print(s); cur_x = 0; cur_y += 8 * cur_size; }

    void display();
    void dim(bool) {}
};

class EEPROM_ {
    std::map<int, byte> storage;
public:
    byte read(int addr) { return storage[addr]; }
    void write(int addr, byte val) { storage[addr] = val; }
};
extern EEPROM_ EEPROM;

class LowPower_ {
public:
    void powerDown(int mode, int adc, int bod) {}
    void sleep(unsigned long ms) {}
};
extern LowPower_ LowPower;

#define ADC_OFF 0
#define BOD_OFF 0
#define SLEEP_FOREVER 0

#endif
