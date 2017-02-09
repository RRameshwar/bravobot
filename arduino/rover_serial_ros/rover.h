#import <Arduino.h>

class Motor {
  int vel;
  bool invert=false;
  
public:
  int cmd(float req_vel) {
    if (req_vel > 1) req_vel = 1;
    if (req_vel < -1) req_vel = -1;
    vel = 90 + req_vel*60;
    if (invert) vel = 180-vel;
    return int(vel);
  }
  void reverse() {
    invert=!invert;
  }
};

class Rover {
  float right_vel;
  float left_vel;
  Motor left_motor;
  Motor right_motor;
public:
  Rover(){
    right_motor.reverse();
    left_motor.reverse();
  }
  void send_cmd(float vel, float turn, int* cmds){
    right_vel = vel+turn;
    left_vel = vel - turn;
    cmds[0] = left_motor.cmd(left_vel);
    cmds[1] = right_motor.cmd(right_vel);
  }
};

