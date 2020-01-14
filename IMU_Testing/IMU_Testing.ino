#include "IMU.h"

IMU_Testing testing;

void setup() {
  testing.setupEverything();
}

void loop() {
  SerialPort.printf("%f\n", testing.calcAngleFromY(true));
}
