#include "neopixels.h"

// LEDs and Indicator Lights
Adafruit_NeoPixel ringFrontLeft  = Adafruit_NeoPixel(12, LED_FRONT_LEFT,  NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel ringFrontRight = Adafruit_NeoPixel(12, LED_FRONT_RIGHT, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripBackLeft  = Adafruit_NeoPixel(8,  LED_BACK_LEFT,   NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripBackRight = Adafruit_NeoPixel(8,  LED_BACK_RIGHT,  NEO_GRBW + NEO_KHZ800);

uint32_t red   = ringFrontLeft.Color(50, 0, 0, 0);
uint32_t white = ringFrontLeft.Color(0, 0, 0, 50);
uint32_t off   = ringFrontLeft.Color(0, 0, 0, 0);

// Millis storing functions for delay purposes
unsigned long previousMillisLED = 0;
unsigned long currentMillisLED;

void setupLEDs() {
  ringFrontLeft.begin();
  ringFrontRight.begin();
  stripBackLeft.begin();
  stripBackRight.begin();
  ringFrontLeft.show();
  ringFrontRight.show();
  stripBackLeft.show();
  stripBackRight.show();
}

// Blinks robot alive indicator light
void softStopLEDs() {
    for (int i = 0; i <= 11; i = i + 2) {
      ringFrontLeft.setPixelColor(i, white);
      ringFrontRight.setPixelColor(i, white);
    }
    for (int i = 1; i <= 11; i = i + 2) {
      ringFrontLeft.setPixelColor(i, red);
      ringFrontRight.setPixelColor(i, red);
    }
    for (int i = 0; i <= 7; i++) {
      stripBackLeft.setPixelColor(i, red);
      stripBackRight.setPixelColor(i, red);
    }
    ringFrontLeft.show();
    ringFrontRight.show();
    stripBackLeft.show();
    stripBackRight.show();
}

void eStopLEDs() {
    currentMillisLED = millis();

    if (currentMillisLED - previousMillisLED >= 100) {
      previousMillisLED = currentMillisLED;

      if (ringFrontLeft.getPixelColor(0) != off) {
        Serial.println("Turning lights off.");
        for (int i = 0; i <= 11; i++) {
          ringFrontLeft.setPixelColor(i, off);
          ringFrontRight.setPixelColor(i, off);
        }
        for (int i = 0; i <= 7; i++) {
          stripBackLeft.setPixelColor(i, off);
          stripBackRight.setPixelColor(i, off);
        }
        ringFrontLeft.show();
        ringFrontRight.show();
        stripBackLeft.show();
        stripBackRight.show();
      } 
      else {
        Serial.println("Turning lights on.");
        for (int i = 0; i <= 11; i++) {
          ringFrontLeft.setPixelColor(i, red);
          ringFrontRight.setPixelColor(i, red);
        }
        for (int i = 0; i <= 7; i++) {
          stripBackLeft.setPixelColor(i, red);
          stripBackRight.setPixelColor(i, red);
        }
        ringFrontLeft.show();
        ringFrontRight.show();
        stripBackLeft.show();
        stripBackRight.show();
      }
    }

//    if (currentMillisLED - previousMillisLED >= 100) {
//      previousMillisLED = currentMillisLED;
//      for (int i = 0; i <= 11; i++) {
//        ringFrontLeft.setPixelColor(i, off);
//        ringFrontRight.setPixelColor(i, off);
//      }
//      for (int i = 0; i <= 7; i++) {
//        stripBackLeft.setPixelColor(i, off);
//        stripBackRight.setPixelColor(i, off);
//      }
//      ringFrontLeft.show();
//      ringFrontRight.show();
//      stripBackLeft.show();
//      stripBackRight.show();
//    }
}

void runLEDs() {
    for (int i = 0; i <= 11; i++) {
      ringFrontLeft.setPixelColor(i, white);
      ringFrontRight.setPixelColor(i, white);
    }
    for (int i = 0; i <= 7; i = i + 2) {
      stripBackLeft.setPixelColor(i, red);
      stripBackLeft.setPixelColor(i, red);
    }
    for (int i = 1; i <= 7; i = i + 2) {
      stripBackLeft.setPixelColor(i, off);
      stripBackRight.setPixelColor(i, off);
    }
    ringFrontLeft.show();
    ringFrontRight.show();
    stripBackLeft.show();
    stripBackRight.show();
}

