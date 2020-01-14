#ifndef _IMU_DMP_H_
#define _IMU_DMP_H_

#include <Wire.h>
#include <Arduino.h>

#define IMU
#define AK8963_SECONDARY
#define COMPASS_ENABLED

extern "C"{
	#include "./util/inv_mpu.h"
	#include "./util/inv_mpu_dmp_motion_driver.h"
}

typedef int inv_error_t;
#define INV_SUCCESS 0
#define INV_ERROR 0x20

enum t_axisOrder{
	X_AXIS, Y_AXIS, Z_AXIS
}

#define UPDATE_ACCEL   (1<<1)
#define UPDATE_GYRO    (1<<2)
#define UPDATE_COMPASS (1<<3)
#define UPDATE_TEMP    (1<<4)

#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW  1
#define INT_LATCHED     1
#define INT_50US_PULSE  0
#define MAX_DMP_SAMPLE_RATE 200
#define FIFO_BUFFER_SIZE 512

const int defaultOrientation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
#define ORIENT_PORTRAIT          0
#define ORIENT_LANDSCAPE         1
#define ORIENT_REVERSE_PORTRAIT  2
#define ORIENT_REVERSE_LANDSCAPE 3

class IMU_DMP
{
	int ax, ay, az;
	int gx, gy, gz;
	int mx, my, mz;
	long qw, qx, qy, qz;
	long temperature;
	unsigned long time;
	float pitch, roll, yaw;
	float heading;

	IMU_DMP();

	inv_error_t begin(void);

	inv_error_t setSensors(unsigned char sensors);

	inv_error_t setGyroFSR(unsigned short fsr);

	unsigned short getGyroFSR(void);

	float getGyroSens(void);

	inv_error_t setAccelFSR(unsigned char fsr);

	unsigned char getAccelFSR(void);

	unsigned short getAccelSens(void);

	unsigned short getMagFSR(void);

	float getMagSens(void);

	inv_error_t setLPF(unsigned short lpf);

	unsigned short getLPF(void);

	inv_error_t setSampleRate(unsigned short rate);

	unsigned short getSampleRate(void);

	inv_error_t setCompassSampleRate(unsigned short rate);

	unsigned short getCompassSampleRate(void);

	bool dataReady();

	inv_error_t update(unsigned char sensors = 
	                   UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

	inv_error_t updateAccel(void);
	inv_error_t updateGyro(void);
	inv_error_t updateCompass(void);
	inv_error_t updateTemperature(void);

	inv_error_t configureFifo(unsigned char sensors);

	unsigned char getFifoConfig(void);

	unsigned short fifoAvailable(void);

	inv_error_t updateFifo(void);

	inv_error_t resetFifo(void);

	inv_error_t enableInterrupt(unsigned char enable = 1);

	inv_error_t setIntLevel(unsigned char active_low);

	inv_error_t setIntLatched(unsigned char enable);

	short getIntStatus(void);

	inv_error_t dmpBegin(unsigned short features = 0, unsigned short fifoRate = MAX_DMP_SAMPLE_RATE);

	inv_error_t dmpLoad(void);

	unsigned short dmpGetFifoRate(void);

	inv_error_t dmpSetFifoRate(unsigned short rate);

	inv_error_t dmpUpdateFifo(void); 

	inv_error_t dmpEnableFeatures(unsigned short mask);

	unsigned short dmpGetEnabledFeatures(void);

	inv_error_t dmpSetTap(unsigned short xThresh = 250, 
	                      unsigned short yThresh = 250, 
	                      unsigned short zThresh = 250,
						  unsigned char taps = 1, 
						  unsigned short tapTime = 100,
						  unsigned short tapMulti = 500);

	bool tapAvailable(void);

	unsigned char getTapDir(void);

	unsigned char getTapCount(void);

	inv_error_t dmpSetOrientation(const signed char * orientationMatrix = defaultOrientation);

	unsigned char dmpGetOrientation(void);

	inv_error_t dmpEnable3Quat(void);

	inv_error_t dmpEnable6Quat(void);

	unsigned long dmpGetPedometerSteps(void);

	inv_error_t dmpSetPedometerSteps(unsigned long steps);

	unsigned long dmpGetPedometerTime(void);

	inv_error_t dmpSetPedometerTime(unsigned long time);

	inv_error_t dmpSetInterruptMode(unsigned char mode);

	inv_error_t dmpSetGyroBias(long * bias);

	inv_error_t dmpSetAccelBias(long * bias);

	inv_error_t lowPowerAccel(unsigned short rate);

	float calcAccel(int axis);

	float calcGyro(int axis);

	float calcMag(int axis);

	float calcQuat(long axis);

	void computeEulerAngles(bool degrees = true);

	float computeCompassHeading(void);

	int selfTest(unsigned char debug = 0);
	
private:
	unsigned short _aSense;
	float _gSense, _mSense;

	float qToFloat(long number, unsigned char q);
	unsigned short orientation_row_2_scale(const signed char *row);
}

#endif // _IMU_DMP_H_
