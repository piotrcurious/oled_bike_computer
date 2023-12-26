// Libraries
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LowPower.h>

// Display settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#define BRIGHTNESS_MAX 255 // Maximum brightness value
#define BRIGHTNESS_MIN 0 // Minimum brightness value
#define BRIGHTNESS_STEP 5 // Brightness decrement step
int brightness = BRIGHTNESS_MAX; // Current brightness value

// Button settings
#define BUTTON_PIN 2 // Pin for the reset button
#define BUTTON_DEBOUNCE 50 // Debounce time in milliseconds
bool buttonState = false; // Current state of the button
bool lastButtonState = false; // Previous state of the button
unsigned long lastDebounceTime = 0; // Last time the button state changed

// Wheel settings
#define WHEEL_PIN 3 // Pin for the wheel sensor
#define WHEEL_DIAMETER 0.66 // Wheel diameter in meters
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI) // Wheel circumference in meters
volatile unsigned long wheelCount = 0; // Number of wheel rotations
volatile unsigned long lastWheelTime = 0; // Last time the wheel rotated
float wheelSpeed = 0; // Current speed in km/h
float tripDistance = 0; // Current trip distance in km
float totalDistance = 0; // Total distance in km
#define WHEEL_TIMEOUT 300000 // Timeout for wheel inactivity in milliseconds
#define WHEEL_MIN_SPEED 0.5 // Minimum speed to consider in km/h
#define WHEEL_DEBOUNCE 10 // Debounce time for wheel sensor in milliseconds
bool wheelState = false; // Current state of the wheel sensor
bool lastWheelState = false; // Previous state of the wheel sensor

// Voltage and current settings // Added this section
#define VOLTAGE_PIN A0 // Pin for the voltage sensor
#define CURRENT_PIN A1 // Pin for the current sensor
#define VOLTAGE_DIVIDER 10 // Voltage divider ratio
#define CURRENT_ZERO 2.5 // Zero current voltage
#define CURRENT_SENSITIVITY 0.066 // Current sensitivity in V/A
float voltage = 0; // Current voltage in V
float current = 0; // Current current in A
float power = 0; // Current power in W
float energy = 0; // Current energy in Wh
float wattsPerKm = 0; // Current watts per km

// EEPROM settings
#include <EEPROM.h>
#define EEPROM_SIZE 4 // Size of EEPROM in bytes
#define EEPROM_ADDR 0 // Starting address of EEPROM

// Sleep settings
#define SLEEP_TIME 8 // Sleep time in seconds
#define SLEEP_CYCLES (WHEEL_TIMEOUT / (SLEEP_TIME * 1000)) // Number of sleep cycles before deep sleep
unsigned int sleepCount = 0; // Number of sleep cycles

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Bicycle Computer");
  display.display();
  delay(2000);

  // Initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, CHANGE);

  // Initialize wheel
  pinMode(WHEEL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelHandler, CHANGE);

  // Initialize voltage and current // Added this line
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  // Read total distance from EEPROM
  totalDistance = readEEPROM();
}

// Loop function
void loop() {
  // Update button state
  updateButton();

  // Update wheel state
  updateWheel();

  // Update wheel speed and distance
  updateSpeedAndDistance();

  // Update voltage and current // Added this line
  updateVoltageAndCurrent();

  // Update display
  updateDisplay();

  // Check if wheel is inactive
  if (millis() - lastWheelTime > WHEEL_TIMEOUT) {
    // Increment sleep count
    sleepCount++;

    // Check if sleep count reached the limit
    if (sleepCount >= SLEEP_CYCLES) {
      // Save trip and total distance to EEPROM
      writeEEPROM(totalDistance);

      // Clear display
      display.clearDisplay();
      display.display();

      // Enter deep sleep mode
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

      // Reset sleep count
      sleepCount = 0;
    }
    else {
      // Enter sleep mode
      LowPower.sleep(SLEEP_TIME * 1000);
    }
  }
  else {
    // Reset sleep count
    sleepCount = 0;
  }
}

// Button interrupt handler
void buttonHandler() {
  // Get the current time
  unsigned long currentTime = millis();

  // Check if the button state changed
  if (currentTime - lastDebounceTime > BUTTON_DEBOUNCE) {
    // Update the last debounce time
    lastDebounceTime = currentTime;

    // Toggle the button state
    buttonState = !buttonState;
  }
}

// Wheel interrupt handler
void wheelHandler() {
  // Get the current time
  unsigned long currentTime = millis();

  // Check if the wheel state changed
  if (currentTime - lastDebounceTime > WHEEL_DEBOUNCE) {
    // Update the last debounce time
    lastDebounceTime = currentTime;

    // Toggle the wheel state
    wheelState = !wheelState;
  }
}

// Update button state
void updateButton() {
  // Check if the button state changed
  if (buttonState != lastButtonState) {
    // Update the last button state
    lastButtonState = buttonState;

    // Check if the button is pressed
    if (buttonState == true) {
      // Reset the trip distance
      tripDistance = 0;

      // Reset the energy // Added this line
      energy = 0;
    }
  }
}

// Update wheel state
void updateWheel() {
  // Check if the wheel state changed
  if (wheelState != lastWheelState) {
    // Update the last wheel state
    lastWheelState = wheelState;

    // Check if the wheel is rotating
    if (wheelState == true) {
      // Increment the wheel count
      wheelCount++;

      // Update the last wheel time
      lastWheelTime = millis();

      // Restore the brightness to maximum
      brightness = BRIGHTNESS_MAX;
    }
  }
}

// Update wheel speed and distance
void updateSpeedAndDistance() {
  // Calculate the wheel speed in km/h
  wheelSpeed = (WHEEL_CIRCUMFERENCE * wheelCount * 3.6) / (millis() - lastWheelTime);

  // Check if the wheel speed is above the minimum threshold
  if (wheelSpeed > WHEEL_MIN_SPEED) {
    // Calculate the trip distance in km
    tripDistance += (WHEEL_CIRCUMFERENCE * wheelCount) / 1000;

    // Calculate the total distance in km
    totalDistance += (WHEEL_CIRCUMFERENCE * wheelCount) / 1000;
  }

  // Reset the wheel count
  wheelCount = 0;
}

// Update voltage and current // Added this function
void updateVoltageAndCurrent() {
  // Read the analog values from the sensors
  int voltageRaw = analogRead(VOLTAGE_PIN);
  int currentRaw = analogRead(CURRENT_PIN);

  // Convert the raw values to voltage and current
  voltage = (voltageRaw * 5.0 / 1023.0) * VOLTAGE_DIVIDER; // Voltage divider formula
  current = ((currentRaw * 5.0 / 1023.0) - CURRENT_ZERO) / CURRENT_SENSITIVITY; // ACS712 formula

  // Calculate the power and energy
  power = voltage * current; // Power formula
  energy += power * (millis() - lastWheelTime) / 3600000.0; // Energy formula

  // Calculate the watts per km
  if (tripDistance > 0) {
    wattsPerKm = energy / tripDistance; // Watts per km formula
  }
  else {
    wattsPerKm = 0;
  }
}

// Update display
void updateDisplay() {
  // Clear display
  display.clearDisplay();

  // Set the display brightness
  display.dim(brightness < BRIGHTNESS_MAX);

  // Display current speed in big font
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(wheelSpeed, 1);
  display.print(" km/h");

  // Display last stored trip in small font in the bottom right corner
  display.setTextSize(1);
  display.setCursor(80, 54);
  display.print("Trip: ");
  display.print(tripDistance, 2);
  display.print(" km");

  // Display total trip counter in bottom left corner
  display.setCursor(0, 54);
  display.print("Total: ");
  display.print(totalDistance, 2);
  display.print(" km");

  // Display the button state
  display.setCursor(64, 54);
  display.print("Reset: ");
  display.print(buttonState ? "ON" : "OFF");

  // Display the watts per km in small font above the trip counter // Added this line
  display.setCursor(80, 40);
  display.print("W/km: ");
  display.print(wattsPerKm, 2);

  // Display the data
  display.display();

  // Decrement the brightness by the step
  brightness -= BRIGHTNESS_STEP;

  // Constrain the brightness to the minimum value
  brightness = max(brightness, BRIGHTNESS_MIN);
}

// Read float value from EEPROM
float readEEPROM() {
  // Create a union of shared memory space
  union {
    float f;
    byte b[4];
  } data;

  // Read 4 bytes from EEPROM into the union
  for (int i = 0; i < 4; i++) {
    data.b[i] = EEPROM.read(EEPROM_ADDR + i);
  }

  // Return the float value
  return data.f;
}

// Write float value to EEPROM
void writeEEPROM(float value) {
  // Create a union of shared memory space
  union {
    float f;
    byte b[4];
  } data;

  // Assign the float value to the union
  data.f = value;

  // Write 4 bytes from the union to EEPROM
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_ADDR + i, data.b[i]);
  }
}
