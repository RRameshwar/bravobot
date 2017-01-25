int ir_threshold=100;

bool check_safe(bool estop, int IR_1, int IR_2){
  if(estop) return true;
  if(IR_1<ir_threshold || IR_2<ir_threshold) return true;
  return false;
}

