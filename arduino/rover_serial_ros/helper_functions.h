#import <Arduino.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define SSLOW 16
#define SLOW 8
#define MID 4
#define FAST 2
#define FFAST 1

int ir_threshold = 400;

bool check_safe(int IR_1, int IR_2) {
  if (IR_1 < ir_threshold || IR_2 < ir_threshold) return true;
  return false;
} 

class Light {

  public:
    int m_r;
    int m_g;
    int m_b=0;
    int m_w;
    int m_pin;
    int m_numLights;
    int m_lights[20];
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

    void LeftOn(int r, int g, int b){
      left.TurnOn(r, g, b);
      right.TurnOn(right.m_r, right.m_g, right.m_b);
      pixels.begin();
      pixels.show();
    }

    void RightOn(int r, int g, int b){
      right.TurnOn(r, g, b);
      left.TurnOn(left.m_r, left.m_g, left.m_b);
      pixels.begin();
      pixels.show();
    }

    void TurnOn(int r, int g, int b){
      right.TurnOn(r, g, b);
      left.TurnOn(r, g, b);
      pixels.begin();
      pixels.show();
    }
};
/*
class StraightLight {
  public:
  Light light;
  Adafruit_NeoPixel pixels;

    void Setup(int pin, int numLights=8) {
      Serial.println("in Setup");
      pixels = Adafruit_NeoPixel(numLights, pin, NEO_RGBW + NEO_KHZ800);
      Serial.println("neopixel setup");
      pinMode(13, OUTPUT);
      
      int all_pixels[numLights+1];
      all_pixels[numLights+1] = 200;
      Serial.println("loop start");
      for (int i=0;i<numLights;i++){
        all_pixels[i] = i;
      }
      Serial.println("loop end");
      light.Setup(all_pixels, pixels);
    }

    void TurnOn(int r, int g, int b){
      light.TurnOn(r, g, b);
      pixels.begin();
      pixels.show();
    }
};
*/
