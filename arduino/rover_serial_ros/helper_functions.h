#import <Arduino.h>

int ir_threshold = 350;

bool check_safe(int IR_1, int IR_2) {
  if (IR_1 < ir_threshold || IR_2 < ir_threshold) return true;
  return false;
} 

void setup_lights(){
  Serial2.begin(9600);
}

void update_lights(bool estop, bool ir_stop, int mode, float vel, float turn){
  char request[6];
  if (estop) request[0]='1'; else request[0]='0';
  if (ir_stop) request[1]='1'; else request[1]='0';
  request[2]=(char) (mode+48);
  if (turn>0.05) request[3]='l';
  else if (turn<-0.05) request[3]='r';
  else request[3]='s';
  if (vel>0.05) request[4]='f';
  else if (vel<-0.05) request[4]='b';
  else request[4]='s';
  request[5]='\0';
  Serial2.print(request);
  Serial2.flush();
}

