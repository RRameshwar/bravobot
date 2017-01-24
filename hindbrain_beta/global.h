//ROS
ros::NodeHandle  nh;

void cmdvel_cb( const geometry_msgs::Twist& msg); //callback prototype, find definition in ROS.ino
ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmdvel_cb);

void enable_cb( const std_msgs::Bool& msg); //callback prototype, find definition in ROS.ino
ros::Subscriber<std_msgs::Bool> enable("hind/enable", &enable_cb);

std_msgs::Int16 portspeed_msg;
std_msgs::Int16 starboardspeed_msg;
ros::Publisher pub_portspeed("motor/port", &portspeed_msg);
ros::Publisher pub_starboardspeed("motor/starboard", &starboardspeed_msg);

std_msgs::String state_msg;
ros::Publisher pub_state("hind/state", &state_msg);

volatile boolean ROSReady = false;

//Motors
Adafruit_TiCoServo port_motors;
Adafruit_TiCoServo starboard_motors;
//Servo port_motors;
//Servo starboard_motors;
//Speeds that the midbrain wants the motors to go
int port_speed_commanded = MOTOR_STOP; // range from 24 to 156
int starboard_speed_commanded = MOTOR_STOP; // range from 24 to 156
//Safe ramping motor values
int port_speed_actual = MOTOR_STOP; // range from 24 to 156
int starboard_speed_actual = MOTOR_STOP; // range from 24 to 156

//Time
long lastPublishTime = 0;
uint32_t last_increment_time = 0;
uint32_t last_LED_refresh = 0;
uint8_t mode;

//Neopixels
Adafruit_NeoPixel led_ring_port  = Adafruit_NeoPixel(12, LED_RING_PORT,  NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel led_ring_starboard = Adafruit_NeoPixel(12, LED_RING_STARBOARD, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel led_strip_port  = Adafruit_NeoPixel(8,  LED_STRIP_PORT,   NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel led_strip_starboard = Adafruit_NeoPixel(8,  LED_STRIP_STARBOARD,  NEO_GRBW + NEO_KHZ800);
uint32_t red   = led_ring_port.Color(50, 0, 0, 0);
uint32_t white = led_ring_port.Color(0, 0, 0, 50);
uint32_t off   = led_ring_port.Color(0, 0, 0, 0);
boolean blinkState = true;

//IR
int IR_PINS[] = {IR_FRONT_PORT, IR_FRONT_STARBOARD, IR_BACK_PORT, IR_BACK_STARBOARD};
int IR_Val[] = {0, 0, 0, 0};




