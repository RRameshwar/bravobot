#include "motors.h"

// Pan-Tilt Servos
int pan  = 90;
int tilt = 90;
Adafruit_TiCoServo panServo;
Adafruit_TiCoServo tiltServo;

// Drive Motors
Adafruit_TiCoServo rightMotors;
Adafruit_TiCoServo leftMotors;

void setupMotors() {
    panServo.attach(7);
    tiltServo.attach(8);
    panServo.write(90);
    tiltServo.write(90);
  
    rightMotors.attach(RIGHT_MOTORS);
    leftMotors.attach(LEFT_MOTORS);
    rightMotors.write(90);
    leftMotors.write(90);
}

void runMotors(int leftMotorRunSpeed, int rightMotorRunSpeed) {
    leftMotors.write(leftMotorRunSpeed);
    rightMotors.write(rightMotorRunSpeed);
}

// Used for e-stop and soft-stop
void stopMotors() {
    leftMotorSpeed = 90;
    rightMotorSpeed = 90;
    prevLeftMotorSpeed = 90;
    prevRightMotorSpeed = 90;

    leftMotors.write(prevLeftMotorSpeed);
    rightMotors.write(prevRightMotorSpeed);
}

