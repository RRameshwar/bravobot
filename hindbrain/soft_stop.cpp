#include "soft_stop.h"

void softStop() {
    stopMotors();
    softStopLEDs();
}
