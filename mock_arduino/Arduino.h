#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <chrono>
#include <stdint.h>

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

class Adafruit_SSD1306 {
public:
    Adafruit_SSD1306(int w, int h, int mosi, int clk, int dc, int reset, int cs) {}
    Adafruit_SSD1306(int w, int h) {}
    Adafruit_SSD1306(int w, int h, int mosi, int clk, int dc, int rst) {}
    void begin(uint8_t) {}
    void clearDisplay() {}
    void setTextSize(uint8_t) {}
    void setTextColor(uint8_t) {}
    void setCursor(int16_t, int16_t) {}
    void print(float, int) {}
    void print(const char*) {}
    void print(float) {}
    void print(int) {}
    void println(const char*) {}
    void display() {}
    void dim(bool) {}
};

class EEPROM_ {
public:
    byte read(int addr) { return 0; }
    void write(int addr, byte val) {}
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
