void setupIR() {
  pinMode(IR_PINS[0], INPUT);
  pinMode(IR_PINS[1], INPUT);
  pinMode(IR_PINS[2], INPUT);
  pinMode(IR_PINS[3], INPUT);
}

boolean is_IR_ok() {  
//  for(int i=0; i<4; i++){
//    if(getRange(IR_PINS[i]) < 50.0) return false;
//  }
//  return true;
  return true;
}

float getRange(int pin){
    int sample;
    sample = analogRead(pin)/4;
    if(sample < 10) //if reading too low, we are far from anything
        return 254;    
    sample= 1309/(sample-3); //get cm
    return (sample - 1)/100; //convert to meters
}
