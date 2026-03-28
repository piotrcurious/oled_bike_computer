#include "mock_arduino/Arduino.h"
#include <iostream>

void buttonHandler();
void wheelHandler();
void updateButton();
void updateWheel();
void updateDisplay();
float readEEPROM();
void writeEEPROM(float value);

#include "speed_odo_trip.ino"

int main() {
    set_millis(0);
    setup();
    display.set_name("odo");

    float speed_kmh = 25.0;
    float speed_mps = speed_kmh / 3.6;
    float pulse_interval_ms = (WHEEL_CIRCUMFERENCE / speed_mps) * 1000.0;
    unsigned long last_pulse = 0;

    for (unsigned long t = 0; t < 2000; t += 10) {
        set_millis(t);
        if (t - last_pulse >= pulse_interval_ms) {
            trigger_interrupt(WHEEL_PIN);
            last_pulse = t;
        }
        loop();
    }

    updateDisplay();
    return 0;
}
