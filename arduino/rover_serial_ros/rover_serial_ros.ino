#include <ArduinoHardware.h>

#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>
/* rover communication code
Uses Rosserial to communicate with a computer running ros
*/

#include "rover.h" //my library for controlling the rover
#include "helper_functions.h" // header file for helper functions

// Include ros library and messages
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//arduino pin config
const int lservo_pin=11;
const int rservo_pin=8;
const int IR_1_pin = A5;
const int IR_2_pin = A4;
const int kill_pin = A3;
const int led_1 = 13;
const int headlight_left=39;

//lights

CircleLight portCircle(38, 0);
//CircleLight starCircle(39, 10);

// ROS setup
ros::NodeHandle nh;

std_msgs::Int16MultiArray ir;
ros::Publisher ir_sensors("ir_sensors", &ir);
std_msgs::Bool is_estopped;
ros::Publisher estop_pub("estop", &is_estopped);
std_msgs::Bool is_irstopped;
ros::Publisher irstop_pub("ir_stop", &is_irstopped); 

int flash_rate=50;

Rover rover; // initialize rover
Adafruit_TiCoServo lservo;
Adafruit_TiCoServo rservo;
int cmds[2]={90,90}; //initialize motor commands to stopped

// Command Velocity Subscriber and Callback
void cmd_vel_cb( const geometry_msgs::Twist& cmd_vel ) {
  float vel = cmd_vel.linear.x;
  float turn = cmd_vel.angular.z;
  rover.send_cmd(vel, turn, cmds);
}
ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmd_vel_cb );

//Blink Rade Subscriber and Callback
void blink_cb(const std_msgs::Int16& blinkRate ){
  flash_rate = blinkRate.data;
}
ros::Subscriber<std_msgs::Int16> flash("blink_rate", &blink_cb);


bool light=false; // led on/off
int count = flash_rate; // blink time (in loop cycles)

bool kill = LOW;
bool estop = LOW;
bool ir_stop = LOW;

int IR_1;
int IR_2;


void setup() {
  // attach servos to pins
  lservo.attach(lservo_pin);
  rservo.attach(rservo_pin);
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(ir_sensors);
  nh.advertise(estop_pub);
  nh.advertise(irstop_pub);
  nh.subscribe(cmd);
  nh.subscribe(flash);

  // memory allocation for the ir list length
  ir.data_length = 2;
  ir.data = (int16_t*) malloc(ir.data_length+1);
  
  pinMode(led_1, OUTPUT); // setup led pin as an output
  pinMode(IR_1_pin, INPUT);
  pinMode(IR_2_pin, INPUT);
  pinMode(kill_pin, INPUT_PULLUP);

  //portCircle.left.TurnOn(0,100,0);
  //starCircle.right.TurnOn(100,0,0);
}

void loop() {
  // runs continuously

  // read IR data
  IR_1 = analogRead(IR_1_pin);
  IR_2 = analogRead(IR_2_pin);
  ir.data[0] = IR_1;
  ir.data[1] = IR_2;
  ir_sensors.publish( &ir );

  // read estop
  estop = analogRead(kill_pin)>500;

  //check kill condition
  ir_stop = check_safe(IR_1, IR_2);

  kill = estop||ir_stop;

  is_irstopped.data=ir_stop;
  is_estopped.data=estop;
  irstop_pub.publish(&is_irstopped);
  estop_pub.publish(&is_estopped);

  // arduino LED blink code
  if (count > flash_rate){ //if count is high enough, toggle the led
    count = 0; //reset count
    light = !light; // toggle led variable
    //digitalWrite(led_1, light); // write to led
  }
  
  if (kill){
    lservo.write(90);
    rservo.write(90);
  }
  else{
    lservo.write(cmds[0]);
    rservo.write(cmds[1]);
  }
   
  nh.spinOnce();
  delay(2); // delay for stability
  count++; // increment count
}


