#include "imu.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void IMU_InitializeIMU(imu* IMU)
{
    IMU->GyroX = 0;
    IMU->GyroY = 0;
    IMU->RollRaw = 0;
    IMU->PitchRaw = 0;
    IMU->YawRaw = 0;
    IMU->RollKal = 0;
    IMU->PitchKal = 0;
    IMU->AccelerationRaw = 0;
}

void IMU_AngleInitializeKalman(angleKalmanFilter* KalmanFilter)
{
    KalmanFilter->Angle = 0;
    KalmanFilter->Bias = 0;
    KalmanFilter->Rate = 0;
    KalmanFilter->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    KalmanFilter->P[0][1] = 0;
    KalmanFilter->P[1][0] = 0;
    KalmanFilter->P[1][1] = 0;

}

void IMU_AngleGetInput(imu* IMU, const mpu6500* MPU6500)
{

    // double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    // double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    IMU->RollRaw = atan2(MPU6500->fAccelY, MPU6500->fAccelZ) * RAD_TO_DEG;
    IMU->PitchRaw = atan(-(MPU6500->fAccelX) / sqrt(MPU6500->fAccelY * MPU6500->fAccelY + MPU6500->fAccelZ * MPU6500->fAccelZ)) * RAD_TO_DEG;

    // could not do this but i want to seperate MPU from Kalman filter and let IMU hold all the values  
    IMU->GyroX = MPU6500->fGyroX;
    IMU->GyroY = MPU6500->fGyroY; 

    IMU->YawRaw = MPU6500->fGyroZ;  
    
}

void IMU_AngleSetKalmanInput(float RawValue, angleKalmanFilter* KalmanFilter) // starting input for kalman filter
{
    KalmanFilter->Angle = RawValue;
}

float IMU_AngleGetKalmanOutput(float GyroValue , float RawValue, angleKalmanFilter* KalmanFilter, float dt) // dt needs to be replaced by loop_time struct 
{   
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    KalmanFilter->Rate = GyroValue - KalmanFilter->Bias;
    KalmanFilter->Angle += dt * KalmanFilter->Rate;  

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    KalmanFilter->P[0][0] += dt * (dt * KalmanFilter->P[1][1] - KalmanFilter->P[0][1] - KalmanFilter->P[1][0] + Q_angle);
    KalmanFilter->P[0][1] -= dt * KalmanFilter->P[1][1];
    KalmanFilter->P[1][0] -= dt * KalmanFilter->P[1][1];
    KalmanFilter->P[1][1] += Q_bias * dt;


    // Calculate angle and bias - Update estimate with measurement zk (RawValue)
    /* Step 3 */
    float Y = RawValue - KalmanFilter->Angle;  //  Angle difference
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = KalmanFilter->P[0][0] + R_measure; //  Estimate error

    /* Step 5 */
    float K[2]; // Roll Kalman gain - This is a 2x1 vector
    K[0] = KalmanFilter->P[0][0] / S;
    K[1] = KalmanFilter->P[1][0] / S;

    /* Step 6 */
    KalmanFilter->Angle  += K[0] * Y;
    KalmanFilter->Bias += K[1] * Y;


    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = KalmanFilter->P[0][0];
    float P01_temp = KalmanFilter->P[0][1];

    KalmanFilter->P[0][0] -= K[0] * P00_temp;
    KalmanFilter->P[0][1] -= K[0] * P01_temp;
    KalmanFilter->P[1][0] -= K[1] * P00_temp;
    KalmanFilter->P[1][1] -= K[1] * P01_temp;


    /* Output */
    return KalmanFilter->Angle;
}

void IMU_VelocityInitializeKalman(velocityKalmanFilter* KalmanFilter)
{
    KalmanFilter->Altitude = 0;
    KalmanFilter->Velocity = 0;
    KalmanFilter->P[0][0] = 1.0f; 
    KalmanFilter->P[0][1] = 0.0f;
    KalmanFilter->P[1][0] = 0.0f;
    KalmanFilter->P[1][1] = 1.0f;
}

void IMU_VelocityGetInput(imu* IMU, const mpu6500* MPU6500, const bme280* BME280)
{
    IMU->Altitude = BME280->altitudeM;
    IMU->AccelerationRaw = -sin(IMU->PitchRaw * DEG_TO_RAD) * MPU6500->fAccelX + cos(IMU->PitchRaw * DEG_TO_RAD) * sin(IMU->RollRaw * DEG_TO_RAD) * MPU6500->fAccelY + cos(IMU->RollRaw * DEG_TO_RAD) * cos(IMU->PitchRaw * DEG_TO_RAD) * MPU6500->fAccelZ;
    IMU->AccelerationRaw = (IMU->AccelerationRaw - 1) * 9.81f;
    IMU->VelocityRaw += IMU->AccelerationRaw * 0.004f;
}

void IMU_VelocitySetKalmanInput(float RawValue, velocityKalmanFilter* KalmanFilter) // starting input for kalman filter
{
    KalmanFilter->Velocity = RawValue;
}

// to jest do naprawy nie wiem o co chodzi możliwe że trzeba dodać GPS i z niego liczyć wysokość drona albo jakąś kamere 
float IMU_VelocityGetKalmanOutput(float RawAcceleration , float RawAltitude, velocityKalmanFilter* KalmanFilter, float dt) // dt needs to be replaced by loop_time struct 
{   
    // Modified by Filip Michalak   
    // Based on http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it and https://github.com/rblilja/AltitudeKF/blob/master/altitude_kf.cpp 
    KalmanFilter->Altitude += KalmanFilter->Velocity * dt +  0.5f * RawAcceleration * dt * dt; // kalman rate = gyrovalue - kalman bias 
    KalmanFilter->Velocity += dt * RawAcceleration;  // angle += dt * rate

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    KalmanFilter->P[0][0] += dt * (dt * KalmanFilter->P[1][1] - KalmanFilter->P[0][1] - KalmanFilter->P[1][0] + Q_accel);
    KalmanFilter->P[0][1] -= dt * KalmanFilter->P[1][1];
    KalmanFilter->P[1][0] -= dt * KalmanFilter->P[1][1];
    KalmanFilter->P[1][1] += Q_accel * dt;


    // Calculate angle and bias - Update estimate with measurement zk (RawValue)
    /* Step 3 */
    float Y = RawAltitude - KalmanFilter->Altitude;  //  raw value - kalman angle  
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = KalmanFilter->P[0][0] + R_measure; //  Estimate error

    /* Step 5 */
    float K[2]; // Roll Kalman gain - This is a 2x1 vector
    K[0] = KalmanFilter->P[0][0] / S;
    K[1] = KalmanFilter->P[1][0] / S;

    /* Step 6 */
    KalmanFilter->Altitude  += K[0] * Y; // angle 
    KalmanFilter->Velocity += K[1] * Y; // bias


    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = KalmanFilter->P[0][0];
    float P01_temp = KalmanFilter->P[0][1];

    KalmanFilter->P[0][0] -= K[0] * P00_temp;
    KalmanFilter->P[0][1] -= K[0] * P01_temp;
    KalmanFilter->P[1][0] -= K[1] * P00_temp;
    KalmanFilter->P[1][1] -= K[1] * P01_temp;


    /* Output */
    return KalmanFilter->Velocity;
}

// ---------------------------------------
// Internal functions
// ---------------------------------------