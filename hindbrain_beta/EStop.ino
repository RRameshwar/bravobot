void setupEStop() {
  pinMode(PHYSICAL_ESTOP,INPUT);
}

boolean is_EStop_ok() {
  return true;
}

