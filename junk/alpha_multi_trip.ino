// Libraries
#include <SPI.h>
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

// Encoder settings
#define ENCODER_A 2 // Interrupt pin for encoder A
#define ENCODER_B 3 // Interrupt pin for encoder B
#define ENCODER_BUTTON 4 // Pin for encoder button
volatile int encoderPos = 0; // Encoder position
volatile bool encoderChanged = false; // Flag for encoder change
int encoderButtonState = 0; // Encoder button state
int encoderButtonLastState = 0; // Encoder button last state
unsigned long encoderButtonLastTime = 0; // Encoder button last time
#define ENCODER_DEBOUNCE 50 // Debounce time for encoder button

// Trip button settings
#define TRIP_BUTTON 5 // Pin for trip button
int tripButtonState = 0; // Trip button state
int tripButtonLastState = 0; // Trip button last state
unsigned long tripButtonLastTime = 0; // Trip button last time
#define TRIP_DEBOUNCE 50 // Debounce time for trip button

// Wheel settings
#define WHEEL_PIN 6 // Interrupt pin for wheel sensor
#define WHEEL_DIAMETER 660.0 // Wheel diameter in mm
volatile int wheelCount = 0; // Wheel count
volatile bool wheelChanged = false; // Flag for wheel change
float wheelSpeed = 0.0; // Wheel speed in km/h
float wheelDistance = 0.0; // Wheel distance in km
unsigned long wheelLastTime = 0; // Wheel last time
#define WHEEL_TIMEOUT 300000 // Timeout for wheel inactivity in ms

// EEPROM settings
#include <EEPROM.h>
#define EEPROM_SIZE 128 // EEPROM size in bytes
#define EEPROM_ADDR 0 // EEPROM address for trip data
#define TRIP_SIZE 4 // Trip data size in bytes
#define TRIP_MAX 30 // Maximum number of trips
#define TRIP_INDEX_ADDR EEPROM_ADDR + TRIP_MAX * TRIP_SIZE // EEPROM address for trip index
int tripIndex = 0; // Current trip index
float tripData[TRIP_MAX]; // Trip data array

// Display options
#define OPTION_MAX 3 // Maximum number of options
int optionIndex = 0; // Current option index
String optionNames[OPTION_MAX] = {"Speed", "Distance", "Cadence"}; // Option names
float optionValues[OPTION_MAX] = {0.0, 0.0, 0.0}; // Option values
#define BIG_FONT_SIZE 3 // Big font size
#define SMALL_FONT_SIZE 1 // Small font size

// Cadence settings
#define CADENCE_PIN 7 // Interrupt pin for cadence sensor
volatile int cadenceCount = 0; // Cadence count
volatile bool cadenceChanged = false; // Flag for cadence change
float cadenceRPM = 0.0; // Cadence RPM
unsigned long cadenceLastTime = 0; // Cadence last time
#define CADENCE_TIMEOUT 5000 // Timeout for cadence inactivity in ms

// Setup function
void setup() {
  // Initialize serial monitor
  Serial.begin(9600);

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(SMALL_FONT_SIZE);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Bicycle Computer");
  display.display();
  delay(1000);

  // Initialize encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  // Initialize trip button
  pinMode(TRIP_BUTTON, INPUT_PULLUP);

  // Initialize wheel sensor
  pinMode(WHEEL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelISR, RISING);

  // Initialize cadence sensor
  pinMode(CADENCE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CADENCE_PIN), cadenceISR, RISING);

  // Load trip data from EEPROM
  loadTripData();
}

// Main loop function
void loop() {
  // Check encoder button
  checkEncoderButton();

  // Check trip button
  checkTripButton();

  // Check wheel sensor
  checkWheelSensor();

  // Check cadence sensor
  checkCadenceSensor();

  // Update display
  updateDisplay();

  // Enter low power mode if wheel is inactive
  if (millis() - wheelLastTime > WHEEL_TIMEOUT) {
    saveTripData();
    display.clearDisplay();
    display.display();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

// Encoder interrupt service routine
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b) {
    encoderPos++;
  } else {
    encoderPos--;
  }
  encoderChanged = true;
}

// Wheel interrupt service routine
void wheelISR() {
  wheelCount++;
  wheelChanged = true;
}

// Cadence interrupt service routine
void cadenceISR() {
  cadenceCount++;
  cadenceChanged = true;
}

// Check encoder button function
void checkEncoderButton() {
  encoderButtonState = digitalRead(ENCODER_BUTTON);
  if (encoderButtonState != encoderButtonLastState) {
    encoderButtonLastTime = millis();
  }
  if ((millis() - encoderButtonLastTime) > ENCODER_DEBOUNCE) {
    if (encoderButtonState == LOW) {
      optionIndex = (optionIndex + 1) % OPTION_MAX;
    }
  }
  encoderButtonLastState = encoderButtonState;
}

// Check trip button function
void checkTripButton() {
  tripButtonState = digitalRead(TRIP_BUTTON);
  if (tripButtonState != tripButtonLastState) {
    tripButtonLastTime = millis();
  }
  if ((millis() - tripButtonLastTime) > TRIP_DEBOUNCE) {
    if (tripButtonState == LOW) {
      saveTripData();
      resetTripData();
    }
  }
  tripButtonLastState = tripButtonState;
}

// Check wheel sensor function
void checkWheelSensor() {
  if (wheelChanged) {
    detachInterrupt(digitalPinToInterrupt(WHEEL_PIN));
    wheelSpeed = 60000.0 / (millis() - wheelLastTime) * WHEEL_DIAMETER * 3.141 * 60 / 1000000;
    wheelDistance += WHEEL_DIAMETER * 3.141 / 1000000;
    wheelLastTime = millis();
    wheelCount = 0;
    wheelChanged = false;
    attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelISR, RISING);
  }
}

// Check cadence sensor function
void checkCadenceSensor() {
  if (cadenceChanged) {
    detachInterrupt(digitalPinToInterrupt(CADENCE_PIN));
    cadenceRPM = 60000.0 / (millis() - cadenceLastTime) * cadenceCount;
    cadenceLastTime = millis();
    cadenceCount = 0;
    cadenceChanged = false;
    attachInterrupt(digitalPinToInterrupt(CADENCE_PIN), cadenceISR, RISING);
  }
}

// Update display function
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(BIG_FONT_SIZE);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(optionNames[optionIndex]);
  display.print(":");
  display.println(optionValues[optionIndex]);
  display.setTextSize(SMALL_FONT_SIZE);
  display.setCursor(SCREEN_WIDTH - 30, SCREEN_HEIGHT - 10);
  display.print("Trip:");
  display.print(tripData[tripIndex]);
  display.display();
}

// Load trip data from EEPROM function
void loadTripData() {
  int addr = EEPROM_ADDR;
  for (int i = 0; i < TRIP_MAX; i++) {
    float value = 0.0;
    EEPROM.get(addr, value);
    tripData[i] = value;
    addr += TRIP_SIZE;
  }
  EEPROM.get(TRIP_INDEX_ADDR, tripIndex); // Load trip index
}

// Save trip data to EEPROM function
void saveTripData() {
  if (tripIndex < TRIP_MAX) {
    int addr = EEPROM_ADDR + tripIndex * TRIP_SIZE;
    EEPROM.put(addr, tripData[tripIndex]);
    tripData[tripIndex] = wheelDistance;
  }
  EEPROM.put(TRIP_INDEX_ADDR, tripIndex); // Save trip index
}

// Reset trip data function
void resetTripData() {
  wheelSpeed = 0.0;
  wheelDistance = 0.0;
  wheelLastTime = millis();
  wheelCount = 0;
  wheelChanged = false;
  tripIndex++; // Increment trip index
}
