#include "mock_arduino/Arduino.h"
#include <iostream>

void buttonHandler();
void wheelHandler();
void updateButton();
void updateWheel();
void updateSpeedAndDistance();
void updateDisplay();
float readEEPROM();
void writeEEPROM(float value);

#include "speed_odo_trip_debounce.ino"

int main() {
    set_millis(0);
    setup();
    display.set_name("deb");

    float speed_kmh = 15.0;
    float speed_mps = speed_kmh / 3.6;
    float pulse_interval_ms = (WHEEL_CIRCUMFERENCE / speed_mps) * 1000.0;
    unsigned long last_pulse = 0;

    for (unsigned long t = 0; t < 2000; t += 10) {
        set_millis(t);
        if (t - last_pulse >= pulse_interval_ms) {
            wheelState = true; trigger_interrupt(WHEEL_PIN);
            last_pulse = t;
        } else if (t - last_pulse >= 50 && wheelState) {
            wheelState = false; trigger_interrupt(WHEEL_PIN);
        }
        loop();
    }

    updateDisplay();
    return 0;
}
