#include <Adafruit_TiCoServo.h>

Adafruit_TiCoServo leftMotors;
Adafruit_TiCoServo rightMotors;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  leftMotors.attach(12);
  rightMotors.attach(13);
  leftMotors.write(90);
  rightMotors.write(90);
  delay(5000);
}

void loop() {  
  // put your main code here, to run repeatedly:
  Serial.println("Motor test loop start.");
  Serial.println("Left motor speed sweep.");
  for (int i = 90; i >= 24; i--){
    leftMotors.write(i);
    rightMotors.write(90);
    Serial.println(i);
    delay(100);
  }
  for (int i = 24; i <= 90; i++){
    leftMotors.write(i);
    rightMotors.write(90);
    Serial.println(i);
    delay(100);
  }

  Serial.println("Right motor speed sweep.");
  for (int i = 90; i >= 24; i--){
    rightMotors.write(i);
    leftMotors.write(90);
    Serial.println(i);
    delay(100);
  }
  for (int i = 24; i <= 90; i++){
    rightMotors.write(i);
    leftMotors.write(90);
    Serial.println(i);
    delay(100);
  }

  Serial.println("Both motors sweep together.");
  for (int i = 90; i >= 24; i--){
    leftMotors.write(i);
    rightMotors.write(i);
    Serial.println(i);
    delay(100);
  }
  for (int i = 24; i <= 90; i++){
    leftMotors.write(i);
    rightMotors.write(i);
    Serial.println(i);
    delay(100);
  }
}
