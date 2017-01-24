/* 
 * hindbrain_beta
 * A hindbrain with a ROSSerial pipe. Oy vey!
 */

#include "includes.h"
#include "defines.h"
#include "pins.h"
#include "global.h"

void setup()
{ 
  //Initialize Hardware
  setupMotors();
  setupROS();
  setupEStop();
  setupIR();
  setupNeopixels();

  mode = SOFT_STOP_MODE;
}

void loop()
{  
  publishAtInterval();
  updateNeopixels();
  
  if(is_EStop_ok() == false){
    mode = PHYS_ESTOP_MODE;
  }
  else if(is_IR_ok() == true && is_ROS_ok() == true){
    mode = OK_MODE;
  }
  else if(is_IR_ok() == false || is_ROS_ok() == false){
    mode = SOFT_STOP_MODE;
  }
  

  switch (mode) {
    case OK_MODE:
      updateRampingSpeed();
      runMotors();
      break;
    case PHYS_ESTOP_MODE:
      stopMotors();
      break;
    case SOFT_STOP_MODE:
      stopMotors();
      break;
    default:
      mode = PHYS_ESTOP_MODE;
      break;
  }

  //ROS
  nh.spinOnce();
  
}


