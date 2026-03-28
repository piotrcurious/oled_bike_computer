#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>

// Forward declarations
void buttonHandler();
void wheelHandler();
void updateButton();
void updateWheel();
void updateDisplay();
float readEEPROM();
void writeEEPROM(float value);

#include "speed_odo_trip.ino"

int main() {
    std::cout << "Testing speed_odo_trip.ino..." << std::endl;

    set_millis(0);
    setup();

    // Wheel rotation 1 at 1s
    set_millis(1000);
    trigger_interrupt(WHEEL_PIN);
    updateWheel();

    // Wheel rotation 2 at 2s
    set_millis(2000);
    trigger_interrupt(WHEEL_PIN);
    updateWheel();

    std::cout << "Speed: " << wheelSpeed << " km/h" << std::endl;
    std::cout << "Trip: " << tripDistance << " km" << std::endl;

    assert(wheelSpeed > 0);
    assert(tripDistance > 0);

    std::cout << "speed_odo_trip.ino tests passed!" << std::endl;
    return 0;
}
