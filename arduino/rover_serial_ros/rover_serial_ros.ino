#include <ArduinoHardware.h>
#include <Servo.h>
/* rover communication code
Uses Rosserial to communicate with a computer running ros
*/

#include "rover.h" //my library for controlling the rover

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

std_msgs::Int16MultiArray ir;
ros::Publisher ir_sensors("ir_sensors", &ir);

float vel;
float turn;
int flash_rate=50;
bool got_msg = false;

Rover rover; // initialize rover with motor pins
Servo lservo;
Servo rservo;
int cmds[2]={90,90};

void cmd_vel_cb( const geometry_msgs::Twist& cmd_vel ) {
  vel = cmd_vel.linear.x;
  turn = cmd_vel.angular.z;
  rover.send_cmd(vel, turn, cmds);
}
ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmd_vel_cb );

void blink_cb(const std_msgs::Int16& blinkRate ){
  flash_rate = blinkRate.data;
}
ros::Subscriber<std_msgs::Int16> flash("blink_rate", &blink_cb);


bool light=false; // led on/off
int count = flash_rate; // blink time (in loop cycles)

const int led_1 = 13;

const int kill_pin = 2;
bool kill = LOW;

const int IR_1_pin = A1;
const int IR_2_pin = A2;
int IR_1;
int IR_2;


void setup() {
  lservo.attach(4);
  rservo.attach(3);
  // runs once at the beginning of the script
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(ir_sensors);
  nh.subscribe(cmd);
  nh.subscribe(flash);
  
  ir.data = (int16_t*) malloc(3);
  ir.data_length = 2;
  
  pinMode(led_1, OUTPUT); // setup pin 12 as an output for the led
  //pinMode(IR_1_pin, INPUT);
  //pinMode(kill_pin, INPUT_PULLUP);
}

void loop() {
  // runs continuously
  
  //kill = digitalRead(kill_pin);
  
  if (count > flash_rate){ //if count is high enough, toggle the led
    count = 0; //reset count
    light = !light; // toggle led variable
    digitalWrite(led_1, light); // write to led
  }
  
  if (kill){
    lservo.write(90);
    rservo.write(90);
  }
  else{
    lservo.write(cmds[0]);
    rservo.write(cmds[1]);
  }
  
  IR_1 = analogRead(IR_1_pin);
  IR_2 = analogRead(IR_2_pin);
  ir.data[0] = IR_1;
  ir.data[1] = IR_2;
  ir_sensors.publish( &ir );
  
  nh.spinOnce();
  delay(2); // delay for stability
  count++; // increment count
}


