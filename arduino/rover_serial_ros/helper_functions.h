int ir_threshold=400;

bool check_safe(int IR_1, int IR_2){
  if(IR_1<ir_threshold || IR_2<ir_threshold) return true;
  return false;
}

