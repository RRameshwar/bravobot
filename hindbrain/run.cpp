#include "run.h"

// Millis storing functions for delay purposes
unsigned long previousMillis = 0;
unsigned long currentMillis;

void run() {

    currentMillis = millis();

    if (currentMillis - previousMillis >= 50) {
      previousMillis = currentMillis;
      
      if (leftMotorSpeed != prevLeftMotorSpeed && leftMotorSpeed > prevLeftMotorSpeed) {
        prevLeftMotorSpeed++;
      }
      if (leftMotorSpeed != prevLeftMotorSpeed && leftMotorSpeed < prevLeftMotorSpeed) {
        prevLeftMotorSpeed--;
      }
      if (rightMotorSpeed != prevRightMotorSpeed && rightMotorSpeed > prevRightMotorSpeed) {
        prevRightMotorSpeed++;
      }
      if (rightMotorSpeed != prevRightMotorSpeed && rightMotorSpeed < prevRightMotorSpeed) {
        prevRightMotorSpeed--;
      }
  
      Serial.println(prevLeftMotorSpeed);
      runMotors(prevLeftMotorSpeed, prevRightMotorSpeed);
      runLEDs();
    }
}
