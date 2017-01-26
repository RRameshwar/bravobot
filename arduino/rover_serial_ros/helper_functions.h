#import <Arduino.h>
#include "Vector.h"
#include <Adafruit_NeoPixel.h>



int ir_threshold = 400;

bool check_safe(int IR_1, int IR_2) {
  if (IR_1 < ir_threshold || IR_2 < ir_threshold) return true;
  return false;
} 

class Light {

  private:
    int m_r;
    int m_g;
    int m_b;
    int m_w;
    int m_pin;
    Vector<int> m_lights;
    Adafruit_NeoPixel m_pixels;

  public:
    void Setup(int pin, Vector<int> lights, Adafruit_NeoPixel pixels, int r = 0, int g = 0, int b = 0, int w = 0) {
      m_r = r;
      m_g = g;
      m_b = b;
      m_w = w;
      m_pin = pin;
      m_lights = lights;
      m_pixels = pixels;
      
    }
    void TurnOn(int r, int g, int b, int w) {
      m_r = r;
      m_b = b;
      m_g = g;
      m_w = w;

      for (int i = 0; i < m_lights.size(); i++) {
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        m_pixels.setPixelColor(m_lights[i], m_pixels.Color(100, 0, 0, 0)); // Moderately bright green color.

        m_pixels.show(); // This sends the updated pixel color to the hardware.
      }

    }
};

class CircleLight {

  Light left;
  Light right;

  private:
   CircleLight(int pin, int offset, int numLights=12) 
    {
      Vector<int> left_pixels;
      Vector<int> right_pixels;
      for(int i=0;i<numLights/2;i++){
        left_pixels.push_back((i+offset)%numLights);
        right_pixels.push_back((i+offset+numLights/2)%numLights);
      }
      if (numLights%2==1){
        right_pixels.push_back((numLights+offset)%12);
      }
      Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numLights, pin, NEO_RGBW + NEO_KHZ800);
      left.Setup(pin, left_pixels, pixels);
      right.Setup(pin, right_pixels, pixels);
    }
};

class StraightLight {
  Light light;

  private:
    StraightLight(int pin, int numLights=6) {
      Vector<int> all_pixels;
      for (int i=0;i<numLights;i++){
        all_pixels.push_back(i);
      }
      Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numLights, pin, NEO_RGBW + NEO_KHZ800);
      light.Setup(pin, all_pixels, pixels);
    }
};

