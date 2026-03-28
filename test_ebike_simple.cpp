#include "mock_arduino/Arduino.h"
#include <iostream>

void buttonHandler();
void wheelHandler();
void updateButton();
void updateWheel();
void updateSpeedAndDistance();
void updateVoltageAndCurrent();
void updateDisplay();
float readEEPROM();
void writeEEPROM(float value);

#include "ebike_simple.ino"

int main() {
    set_millis(0);
    setup();
    display.set_name("ebike");

    unsigned long time = 0;
    float current_speed_kmh = 20.0;
    float speed_mps = current_speed_kmh / 3.6;
    float pulse_interval_ms = (WHEEL_CIRCUMFERENCE / speed_mps) * 1000.0;
    unsigned long last_pulse_time = 0;

    // Run for 2 seconds to get stable speed
    for (time = 0; time < 2000; time += 10) {
        set_millis(time);
        if (time - last_pulse_time >= pulse_interval_ms) {
            wheelState = true; trigger_interrupt(WHEEL_PIN);
            last_pulse_time = time;
        } else {
            wheelState = false; trigger_interrupt(WHEEL_PIN);
        }
        set_analog_input(VOLTAGE_PIN, 700);
        set_analog_input(CURRENT_PIN, 600);
        loop();
    }

    // Capture one frame at 2s
    updateDisplay();

    return 0;
}
