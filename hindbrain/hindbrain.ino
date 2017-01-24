//BravoBot Hindbrain
//Version 2.0 12/01/2016
//Major Revisions: Changed architecture to FSM

#include "hindbrain.h"

int leftMotorSpeed = 90;
int rightMotorSpeed = 90;
int prevLeftMotorSpeed = 90;
int prevRightMotorSpeed = 90;

// For testing only
int testDirection = 1; //1 is forward, 0 is backward

// Initialize Hindbrain State
char command = 'g'; // Command from midbrain
                    // 'g' = run
                    // 's' = software stop
                    // 'p' = rc passthrough
char state   = 'r'; // Hindbrain state
                    // 'r' = run
                    // 's' = software stop
                    // 'e' = e-stop
                    // 'p' = rc passthrough

void setup() {
  Serial.begin(9600);

  pinMode(ESTOP, INPUT);

  setupLEDs();
  setupMotors();

  command = 's'; // default to software stopped mode for safety
}

void loop() {

  // Temporary serial buffer read code to test state switching for FSM
  // "Reading" commmands from midbrain
  while (Serial.available()) {
    command = Serial.read();
    delay(2);
  }

//  Serial.print("Commmand from midbrain: ");
//  Serial.println(command);

  // Temporary to be replaced with actual reading from e-stop relay
  if (command == 'e') {
    state = 'e';
  }

  switch (command) {
    case 'g':
      state = 'r';
      break;
    case 'e':
      state = 'e';
      break;
    case 's':
      state = 's';
      break;
    case 'p':
      state = 'p';
      break;
    default:
      state = 's';
      break;
  }

//  Serial.print("Hindbrain state: ");
//  Serial.println(state);

  switch (state) {
    case 'r':
      run();
      break;
    case 'e':
      eStop();
      break;
    case 's':
      softStop();
      break;
    case 'p':
      rcPassthrough();
      break;
  }

  leftMotorSpeed = 24;
  rightMotorSpeed = 24;
  
  // Test code that ramps motor speeds for run();
//  if (testDirection == 1) {
//    leftMotorSpeed--;
//    rightMotorSpeed--;
//  }
//  if (testDirection == 0) {
//    leftMotorSpeed++;
//    rightMotorSpeed++;
//  }
//  if (leftMotorSpeed == 24 || rightMotorSpeed == 24) {
//    testDirection = 0;
//  }
//  if (leftMotorSpeed == 156 || rightMotorSpeed == 156){
//    testDirection = 1;
//  }
}
