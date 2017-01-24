void setupMotors() {
  starboard_motors.attach(STARBOARD_MOTORS);
  port_motors.attach(PORT_MOTORS);
}

void runMotors() {
  port_motors.write(port_speed_actual);
  starboard_motors.write(starboard_speed_actual);
}

void stopMotors() {
  port_motors.write(MOTOR_STOP);
  starboard_motors.write(MOTOR_STOP);
  port_speed_actual = MOTOR_STOP;
  starboard_speed_actual = MOTOR_STOP;
  port_speed_commanded = MOTOR_STOP;
  starboard_speed_commanded = MOTOR_STOP;
}

void updateRampingSpeed(){
  if(millis() - last_increment_time > MOTOR_INCREMENT_DELAY){
    last_increment_time = millis();
    if(port_speed_commanded > port_speed_actual) port_speed_actual++;
    else if(port_speed_commanded < port_speed_actual) port_speed_actual--;
    if(starboard_speed_commanded > starboard_speed_actual) starboard_speed_actual++;
    else if(starboard_speed_commanded < starboard_speed_actual) starboard_speed_actual--; 
    char pMsg[8]; 
    char sMsg[8];  
    dtostrf(port_speed_actual, 6, 2, pMsg); //Converts floats (and ints) to char arrays, which are required by loginfo.
    dtostrf(starboard_speed_actual, 6, 2, sMsg);
    nh.loginfo("Motor speed changed. New speed (port, starboard):");
    nh.loginfo(pMsg);
    nh.loginfo(sMsg);
  }
}
//setMotorSpeed: Converts linar and angular speeds between -1 and 1 to Motor servo values from 24-90-156. These values are assigned to
//global variables that handle motor speed (allowing speed to be changed in the loop()).
void setMotorSpeed(float linearSpeed, float angularDir) {
  if(mode != OK_MODE){ //don't change motor control values if not in OK_MODE
    port_speed_commanded = MOTOR_STOP;
    starboard_speed_commanded = MOTOR_STOP;
    return;
  }
  //If angular direction = 0, then we send linear speed * r to both motors.
  //If angular dirction = -1 then we send linear speed * r to the right motors, and 85 to the left.
  //If angular dirction =  1 then we send linear speed * r to the left motors, and 85 to the right.
  //If angluar speed = ang_speed, then se send linear speed*r to the left motors, and linear speed *r * (1-ang_speed) to the right.
  //If angluar speed = -ang_speed, then se send linear speed*r to the right motors, and linear speed *r * (1-ang_speed) to the right.

  linearSpeed = constrain(linearSpeed,-1.0,1.0); //failsafe to prevent wackiness if values are unconstrained
  angularDir = constrain(angularDir,-1.0,1.0);   //failsafe to prevent wackiness if values are unconstrained
  
  //Helper variables that designate motor speeds, but at the scale (i.e. -1 to 1) of the input values
  float port_speed_rel; // range from -1 to 1
  float starboard_speed_rel; // range from -1 to 1

  //Calculate motors' differential speeds assigned to a scale of -1 to 1.
  if (linearSpeed >= 0){ //we gon go forward!
    if (angularDir == 0) { //straight
        port_speed_rel = linearSpeed;
        starboard_speed_rel = linearSpeed;
    }
    else if (angularDir>0){//right, right motor should go faster
        port_speed_rel = linearSpeed;
        starboard_speed_rel = linearSpeed*(1-abs(angularDir))+.005; //added the abs()
        
    }
    else if (angularDir<0){//left
        starboard_speed_rel = linearSpeed;
        port_speed_rel = linearSpeed*(1-abs(angularDir))+0.005; //added the abs() 
    }
  }
  else {//we gon go straight backwards
      starboard_speed_rel = linearSpeed;
      port_speed_rel = linearSpeed;
  }

  //Map relative speeds to actual motor values and update the global variables.
  port_speed_commanded = round(port_speed_rel*MOTOR_MULTIPLIER + MOTOR_OFFSET);
  starboard_speed_commanded = round(starboard_speed_rel*MOTOR_MULTIPLIER + MOTOR_OFFSET);
  
}

