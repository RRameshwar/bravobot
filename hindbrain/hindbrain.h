#ifndef HINDBRAIN_H
#define HINDBRAIN_H

#include "Arduino.h"
#include "pins.h"

#include "neopixels.h"
#include "motors.h"

#include "run.h"
#include "e_stop.h"
#include "soft_stop.h"
#include "rc_passthrough.h"

extern int leftMotorSpeed;
extern int rightMotorSpeed;
extern int prevLeftMotorSpeed;
extern int prevRightMotorSpeed;

#endif
