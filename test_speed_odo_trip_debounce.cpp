#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>

// Forward declarations
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
    std::cout << "Testing speed_odo_trip_debounce.ino..." << std::endl;

    set_millis(0);
    setup();

    // First pulse at 1000ms
    set_millis(1000);
    wheelState = true;
    updateWheel(); // updates lastWheelTime to 1000, lastWheelTime_prev to 0
    updateSpeedAndDistance(); // count=1, lastTime=1000, lastTime_prev=0, interval=1000

    std::cout << "Speed after 1st rotation: " << wheelSpeed << " km/h" << std::endl;
    assert(wheelSpeed > 0);
    assert(tripDistance > 0);

    // Second pulse at 1500ms
    set_millis(1500);
    wheelState = false; // reset
    updateWheel();
    wheelState = true;
    updateWheel(); // interval = 500ms
    updateSpeedAndDistance();

    std::cout << "Speed after 2nd rotation: " << wheelSpeed << " km/h" << std::endl;
    assert(wheelSpeed > 10); // interval is smaller, speed should be higher

    std::cout << "speed_odo_trip_debounce.ino tests passed!" << std::endl;
    return 0;
}
