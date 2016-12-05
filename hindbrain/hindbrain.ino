//BravoBot Hindbrain
//Version 2.0 12/01/2016
//Major Revisions: Changed architecture to FSM

#include "hindbrain.h"
#include "run.h"
#include "e_stop.h"
#include "soft_stop.h"
#include "rc_passthrough.h"

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  pinMode(ESTOP, INPUT);

  panServo.attach(7);
  tiltServo.attach(8);
  panServo.write(90);
  tiltServo.write(90);

  rightMotors.attach(9);
  leftMotors.attach(10);
  rightMotors.write(90);
  leftMotors.write(90);

  ringFrontLeft.begin();
  ringFrontRight.begin();
  stripBackLeft.begin();
  stripBackRight.begin();
  ringFrontLeft.show();
  ringFrontRight.show();
  stripBackLeft.show();
  stripBackRight.show();

  command = 's';
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

  if (readEstop()) {
    state = 'e';
  }

  switch (command) {
    case 'g':
      state = 'r';
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
}

// Reads relay attached to robot e-stop switch
boolean readEstop() {
  boolean eStopTriggered = digitalRead(ESTOP);
  return eStopTriggered;
}
