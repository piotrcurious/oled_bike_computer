#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>

// Forward declarations
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
    std::cout << "Testing ebike_simple.ino..." << std::endl;

    set_millis(0);
    setup();

    // Wheel rotation 1 at 1000ms
    set_millis(1000);
    wheelState = true;
    updateWheel(); // updates lastWheelTime to 1000, lastWheelTime_prev to 0
    updateSpeedAndDistance(); // count=1, interval=1000ms

    std::cout << "Speed after 1st rotation: " << wheelSpeed << " km/h" << std::endl;
    assert(wheelSpeed > 0);
    assert(tripDistance > 0);

    // Wheel rotation 2 at 1500ms
    set_millis(1500);
    wheelState = false; // reset
    updateWheel();
    wheelState = true;
    updateWheel(); // interval = 500ms
    updateSpeedAndDistance();

    std::cout << "Speed after 2nd rotation: " << wheelSpeed << " km/h" << std::endl;
    assert(wheelSpeed > 10); // interval is smaller, speed should be higher

    // Reset test
    buttonState = true;
    updateButton();
    assert(tripDistance == 0);

    std::cout << "ebike_simple.ino tests passed!" << std::endl;
    return 0;
}
