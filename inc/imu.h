#ifndef IMU_MODULE_
#define IMU_MODULE_

#include<stdio.h>
#include"pico/stdlib.h"
#include"mpu6500.h"
#include<math.h>

// ---------------------------------------
// const Calibration data
// ---------------------------------------
#define Q_angle 0.001f // can do 0.005f or 0.001f to test 
#define Q_bias 0.003f
#define R_measure 0.04f // could do 0.05f or 0.03f to test 

#define RAD_TO_DEG (3.142 / 180)
// ---------------------------------------
// IMU struct
// ---------------------------------------
typedef struct 
{
    // rate of quadcopter 
    float GyroX, GyroY;
    // angles of quadcopter
    float RollRaw, PitchRaw, YawRaw; 
    // kalman output 
    float RollKal, PitchKal;  
}imu;

typedef struct 
{
    float RollAngle , RollBias, RollRate, RollP[2][2]; 
    float PitchAngle, PitchBias, PitchRate, PitchP[2][2];
}kalmanfilter;

// ---------------------------------------
// Public API
// ---------------------------------------
void IMU_InitializeIMU(imu* IMU);
void IMU_InitializeKalman(kalmanfilter* KalmanFilter);
void IMU_GetInput(imu* IMU, const mpu6500* MPU6500); 
void IMU_GetKalmanOutput(imu* IMU, kalmanfilter* KalmanFilter, float dt);
void IMU_SetKalmanInput(imu* IMU, kalmanfilter* KalmanFilter);
// ---------------------------------------
// Internal functions
// ---------------------------------------

#endif // IMU_MODULE_