#import <Arduino.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

class Light {

  public:
    int m_r;
    int m_g;
    int m_b=0;
    int m_w;
    int m_pin;
    int m_numLights;
    int m_lights[24];
    Adafruit_NeoPixel m_pixels;

    void Setup(int lights[20], Adafruit_NeoPixel pixels, int r = 0, int g = 0, int b = 0, int w = 0) {
      m_r = r;
      m_g = g;
      m_b = b;
      m_w = w;
      m_pixels = pixels;
      m_pixels.begin();
      m_pixels.show();
      int i;
      for (i=0;lights[i]<100;i++){
        m_lights[i] = lights[i];
      }
      m_numLights = i;
    }
    void TurnOn(int r, int g, int b, int w=0) {
      m_r = r;
      m_b = b;
      m_g = g;
      m_w = w;
      int light;
      for (int i = 0; i < m_numLights; i++) {
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        light=m_lights[i];
        m_pixels.setPixelColor(light, m_pixels.Color(m_g, m_r, m_b, m_w));
      }
    }
};

class CircleLight {

public:

  Light left;
  Light right;
  Adafruit_NeoPixel pixels;

   void Setup(int pin, int offset, int numLights=12) 
    {
      pixels = Adafruit_NeoPixel(numLights, pin, NEO_RGBW + NEO_KHZ800);
      pinMode(13, OUTPUT);

      int left_pixels[numLights/2+1];
      int right_pixels[numLights-numLights/2+1];
      left_pixels[numLights/2+1]=200;
      right_pixels[numLights-numLights/2+1]=200;
      
      int i;
      for(i=0;i<numLights/2;i++){
        left_pixels[i] = (i+offset)%numLights;
      }
      left.Setup(left_pixels, pixels);
      for(int j=i;j<numLights;j++){
        right_pixels[j-numLights/2]=((j+offset)%numLights);
      }
      
      right.Setup(right_pixels, pixels);
    }

    void LeftOn(int r, int g, int b, int w=0){
      left.TurnOn(r, g, b, w);
      right.TurnOn(right.m_r, right.m_g, right.m_b, right.m_w);
      pixels.begin();
      pixels.show();
    }

    void RightOn(int r, int g, int b, int w=0){
      right.TurnOn(r, g, b, w);
      left.TurnOn(left.m_r, left.m_g, left.m_b, left.m_w);
      pixels.begin();
      pixels.show();
    }

    void TurnOn(int r, int g, int b, int w=0){
      right.TurnOn(r, g, b, w);
      left.TurnOn(r, g, b, w);
      pixels.begin();
      pixels.show();
    }
};

#define SSLOW 16
#define SLOW 8
#define MID 4
#define FAST 2
#define FFAST 1
#define SOLID -1

class roverLights {
  CircleLight portFore;
  CircleLight starFore;
  CircleLight portAft;
  CircleLight starAft;
  int pf[2];
  int sf[2];
  int pr[2];
  int sr[2];

  int cycle_count=0;
  bool status_lights = LOW;
  bool left_nav_lights = LOW;
  bool right_nav_lights = LOW;

  public:
  int status_speed = SLOW;
  int left_nav_speed = SOLID;
  int right_nav_speed = MID;
  char* dir="ss";
  char* last_dir="no";

  int status_on[4]={0, 0, 0, 50};
  int status_blink[4]={0, 0, 200, 50};
  int rear_nav_on[4]={100, 0, 0, 0};
  int rear_nav_blink[4]{100, 50, 0, 0};

  roverLights(int pf_pin, int pf_off, int sf_pin, int sf_off, int pr_pin, int pr_off, int sr_pin, int sr_off){
    pf[0] = pf_pin; pf[1] = pf_off;
    sf[0] = sf_pin; sf[1] = sf_off;
    pr[0] = pr_pin; pr[1] = pr_off;
    sr[0] = sr_pin; sr[1] = sr_off;
  }
  void Setup(){
    portFore.Setup(pf[0], pf[1]);
    starFore.Setup(sf[0], sf[1]);
    portAft.Setup(pr[0], pr[1]);
    starAft.Setup(sr[0], sr[1]);
  
    portFore.LeftOn(100, 0, 0);
    starFore.RightOn(0, 100, 0);
    //portAft.TurnOn(100, 0, 0);
    //starAft.TurnOn(100, 0, 0);
  }
  void ForeOn(int r, int g, int b){
    portFore.RightOn(r, g, b);
    starFore.LeftOn(r, g, b);
  }
  void AftOutOn(int r, int g, int b, int w=0){
    portAft.RightOn(r, g, b);
    starAft.RightOn(r, g, b);
  }
  void AftInOn(int r, int g, int b, int w=0){
    portAft.LeftOn(r, g, b, w);
    starAft.LeftOn(r, g, b, w);
  }

  void FailsafeSet(bool estop, bool ir_stop){
    if (ir_stop&&(left_nav_speed!=FFAST)){
      status_speed=FFAST;
      left_nav_speed=FFAST;
      right_nav_speed=FFAST;
      last_dir[0]='n';
    }
    else if (estop&&(status_speed!=SOLID)){
      status_speed=SOLID;
    }
  }

  void SetBlink(bool estop, bool ir_stop){
    if (!ir_stop){
      if(status_speed!=SLOW&&!estop){
        status_speed=SLOW;
      }
      if (last_dir[0]!=dir[0]){
        this->SetNav();
        last_dir[0]=dir[0];
      }
    }
  }

  void Cycle(bool estop, bool ir_stop, int mode, char* dir_in){
    dir = dir_in;
    this->FailsafeSet(estop, ir_stop);
    this->SetBlink(estop, ir_stop);
    this->SetStatusColors(mode);
    this->SetNavColors(dir[1]);
    if (cycle_count%left_nav_speed==0){
      left_nav_lights = !left_nav_lights;
      this->SetLeftNav();
    }
    if (cycle_count%right_nav_speed==0){
      right_nav_lights = !right_nav_lights;
      this->SetRightNav();
    }
    if (cycle_count%status_speed==0){
      status_lights = !status_lights;
      this->SetStatus();
    }
    cycle_count = (cycle_count+1)%SSLOW;
  }

  void SetColors(int old_on[4], int new_on[4], int old_blink[4], int new_blink[4]){
    for (int i=0;i<4;i++){
        old_on[i]=new_on[i];
        old_blink[i]=new_blink[i];
      }
  }

  void SetStatusColors(int mode){
    int new_status_on[4]={0,0,0,0};
    int new_status_blink[4]={0,0,0,0};
    switch (mode){
    case '0':
      new_status_blink[3]=50;
      new_status_blink[2]=50;
      new_status_on[2]=200;
      break;
    case '1':
      new_status_on[0]=50;
      new_status_on[2]=50;
      new_status_blink[3]=50;
      new_status_blink[2]=50;
      break;
    case '2':
      new_status_on[0]=70;
      new_status_on[1]=30;
      new_status_blink[3]=50;
      new_status_blink[0]=30;
      break;
    case '3':
      new_status_on[1]=100;
      new_status_blink[3]=50;
      break;
    case '4':
      new_status_on[0]=100;
      new_status_blink[3]=50;
      break;
    case '5':
      new_status_on[2]=10;
      new_status_blink[2]=200;
      break;
    case '6':
      new_status_blink[0]=50;
      new_status_blink[2]=50;
      new_status_on[0]=5;
      new_status_on[2]=5;
      break;
    case'7':
      new_status_blink[0]=70;
      new_status_blink[1]=30;
      new_status_on[0]=7;
      new_status_on[1]=3;
      break;
    case '8':
      new_status_blink[1]=100;
      new_status_on[1]=20;
      break;
    case '9':
      new_status_blink[0]=100;
      new_status_on[0]=10;
      break;
    }
    this->SetColors(status_on, new_status_on, status_blink, new_status_blink);
  }
  void SetNavColors(int dir){
    int new_rear_nav_on[4]={0, 0, 0, 0};
    int new_rear_nav_blink[4]={100, 50, 0, 0};
    switch (dir){
      case 's':
      new_rear_nav_on[0]=100;
      break;
      case 'b':
      new_rear_nav_on[3]=50;
      break;
    }
    this->SetColors(rear_nav_on, new_rear_nav_on, rear_nav_blink, new_rear_nav_blink);
  }


  void SetNav(){
    if (dir[0]=='s') this->Straight();
    else if (dir[0]=='l') {
      this->LeftBlink();
    }
    else if (dir[0]=='r') this->RightBlink();
  }

  void SetLeftNav(){
    if (left_nav_lights||left_nav_speed==SOLID){
      portFore.LeftOn(100, 0, 0);
      portAft.RightOn(rear_nav_on[0], rear_nav_on[1], rear_nav_on[2], rear_nav_on[3]);
    }
    else{
      portFore.LeftOn(0, 0, 0);
      portAft.RightOn(rear_nav_blink[0], rear_nav_blink[1], rear_nav_blink[2], rear_nav_blink[3]);
    }
  }
  void SetRightNav(){
    if (right_nav_lights||right_nav_speed==SOLID){;
      starFore.RightOn(0, 100, 0);
      starAft.RightOn(rear_nav_on[0], rear_nav_on[1], rear_nav_on[2], rear_nav_on[3]);
    }
    else{
      starFore.RightOn(0, 0, 0);
      starAft.RightOn(rear_nav_blink[0], rear_nav_blink[1], rear_nav_blink[2], rear_nav_blink[3]);
    }
  }
  void SetStatus(){
    if (status_lights||status_speed==SOLID){
      portFore.RightOn(status_on[0], status_on[1], status_on[2], status_on[3]);
      starFore.LeftOn(status_on[0], status_on[1], status_on[2], status_on[3]);
      AftInOn(status_on[0], status_on[1], status_on[2], status_on[3]);
    }
    else{
      portFore.RightOn(status_blink[0], status_blink[1], status_blink[2], status_blink[3]);
      starFore.LeftOn(status_blink[0], status_blink[1], status_blink[2], status_blink[3]);
      AftInOn(status_blink[0], status_blink[1], status_blink[2], status_blink[3]);
    }
  }
  
  void LeftBlink(){
    left_nav_speed=MID;
    right_nav_speed=SOLID;
  }
  void RightBlink(){
    left_nav_speed=SOLID;
    right_nav_speed=MID;
  }
  void Straight(){
    left_nav_speed=SOLID;
    right_nav_speed=SOLID;
  }
  
};
