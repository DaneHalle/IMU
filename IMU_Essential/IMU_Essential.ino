#include "IMU.h"
#define SerialPort SerialUSB
IMU data;

void setup() {}

void loop() {
  SerialPort.println(data.calcAngleFromY(true));
}
