void setupNeopixels() {
  led_ring_port.begin();
  led_ring_starboard.begin();
  led_strip_port.begin();
  led_strip_starboard.begin();
  led_ring_port.show();
  led_ring_starboard.show();
  led_strip_port.show();
  led_strip_starboard.show();
}

void updateNeopixels() {
  switch (mode) {
    case OK_MODE:
      LED_ok();
      break;
    case PHYS_ESTOP_MODE:
      LED_estop();
      break;
    case SOFT_STOP_MODE:
      LED_softstop();
      break;
    default:
      LED_softstop();
      break;
  }
  led_ring_port.show();
  led_ring_starboard.show();
  led_strip_port.show();
  led_strip_starboard.show();
}

void LED_ok(){
  for (int i = 0; i <= 11; i++) {
      led_ring_port.setPixelColor(i, white);
      led_ring_starboard.setPixelColor(i, white);
    }
  for (int i = 0; i <= 7; i = i + 2) {
    led_strip_port.setPixelColor(i, red);
    led_strip_starboard.setPixelColor(i, red);
  }
  for (int i = 1; i <= 7; i = i + 2) {
    led_strip_port.setPixelColor(i, off);
    led_strip_starboard.setPixelColor(i, off);
  }
}

void LED_estop(){
  if(last_LED_refresh - millis() > LED_BLINK_INTERVAL) {
      blinkState = !blinkState; 
      last_LED_refresh = millis();
  }
  if(blinkState) {
      for (int i = 0; i <= 11; i++) {
          led_ring_port.setPixelColor(i, red);
          led_ring_starboard.setPixelColor(i, red);
        }
      for (int i = 0; i <= 7; i++) {
        led_strip_port.setPixelColor(i, red);
        led_strip_starboard.setPixelColor(i, red);
      }
  }
  else {
      for (int i = 0; i <= 11; i++) {
          led_ring_port.setPixelColor(i, off);
          led_ring_starboard.setPixelColor(i, off);
        }
      for (int i = 0; i <= 7; i++) {
        led_strip_port.setPixelColor(i, off);
        led_strip_starboard.setPixelColor(i, off);
      }
  }
}

void LED_softstop(){
  for (int i = 0; i <= 11; i = i + 2) {
      led_ring_port.setPixelColor(i, white);
      led_ring_starboard.setPixelColor(i, white);
    }
  for (int i = 1; i <= 11; i = i + 2) {
    led_ring_port.setPixelColor(i, red);
    led_ring_starboard.setPixelColor(i, red);
  }
  for (int i = 0; i <= 7; i++) {
    led_strip_port.setPixelColor(i, red);
    led_strip_starboard.setPixelColor(i, red);
  }
}

