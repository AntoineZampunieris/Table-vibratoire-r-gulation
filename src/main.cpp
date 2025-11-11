#include <Arduino.h>

// PWM pins for the six motors
const int motorPins[6] = {3, 4, 5, 6, 7, 8};

// Motor speed values (0–255)
int motorSpeeds[6] = {150, 150, 150, 150, 150, 150};

void setup() {
  Serial.begin(115200);
  Serial.println("6-Motor PWM Control - Arduino Zero");

  // Initialize pins as outputs
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], motorSpeeds[i]);  // Start all motors off
  }
}

void loop() {

}
