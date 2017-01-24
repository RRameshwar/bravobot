#include "e_stop.h"

void eStop() {
    stopMotors();
    eStopLEDs();
}

// Reads relay attached to robot e-stop switch
boolean readEstop() {
  boolean eStopTriggered = digitalRead(ESTOP);
  return eStopTriggered;
}
