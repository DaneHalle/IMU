#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#define SerialPort SerialUSB

class IMU {
  public:
    IMU();
    void setupEverything();
    void setupOrient();
    void setupGyro();
    void setupAccel();
    void printOrientData();
    void printGyroData();
    void printAccelData();
    float getQX();
    float getQY();
    float getQZ();
    float getRollRad();
    float getPitchRad();
    float getYawRad();
    float getRollDeg();
    float getPitchDeg();
    float getYawDeg();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    void checkOrient();
    void checkGyro();
    void checkAccel();
    float calcAngleFromX(bool);
    float calcAngleFromY(bool);

  private: 
    static const signed char orientationMatrix[];
    static unsigned char lastOrient;

};

#endif
