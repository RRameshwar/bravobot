#include <Adafruit_NeoPixel.h>
#include "light_functions.h"

roverLights lights(8, 8, 9, 4, 10, 4, 11, 4);
byte led_1=13;
bool led=LOW;
int count = 0;
int flash_rate=50;
bool estop = LOW;
bool ir_stop = LOW;
char* dir="ss";
int mode='0';

void setup() {
  // put your setup code here, to run once:
  lights.Setup();
  Serial.begin(9600);
}

void loop() {
  if (count > flash_rate){ //if count is high enough, toggle the led
    count = 0; //reset count
    //digitalWrite(led_1, led); // write to led
    //led = !led;
    lights.Cycle(estop, ir_stop, mode, dir);
  }
  if (count==flash_rate-5){
    int bytes = Serial.available();
    if (bytes>0){
      char msg[bytes+1];
      Serial.readBytes(msg, bytes);
      if (bytes==5){
        estop = msg[0]=='1';
        ir_stop = msg[1]=='1';
        dir[0] = msg[3];
        dir[1] = msg[4];
        mode = int(msg[2]);
        digitalWrite(led_1, led); // write to led
        led = !led;
        //Serial.print(1);

      }
      //else Serial.print(0);
    }
  }
  delay(2);
  count++;
}
