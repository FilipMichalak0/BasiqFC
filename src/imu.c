#include"imu.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void IMU_InitializeIMU(imu* IMU)
{
    IMU->gyroX = 0;
    IMU->GyroY = 0;
    IMU->RollRaw = 0;
    IMU->PitchRaw = 0;
    IMU->YawRaw = 0;
    IMU->RollKal = 0;
    IMU->PitchKal = 0;
}

void IMU_InitializeKalman(kalmanfilter* KalmanFilter)
{
    KalmanFilter->RollAngle = 0;
    KalmanFilter->RollBias = 0;
    KalmanFilter->RollRate = 0;
    KalmanFilter->RollP[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    KalmanFilter->RollP[0][1] = 0;
    KalmanFilter->RollP[1][0] = 0;
    KalmanFilter->RollP[1][1] = 0;
    KalmanFilter->PitchAngle = 0;
    KalmanFilter->PitchBias = 0;
    KalmanFilter->PitchRate = 0;
    KalmanFilter->PitchP[0][0] = 0; 
    KalmanFilter->PitchP[0][1] = 0;
    KalmanFilter->PitchP[1][0] = 0;
    KalmanFilter->PitchP[1][1] = 0;
}

void IMU_GetInput(imu* IMU, const mpu6500* MPU6500)
{
    // Getting this value from this video https://www.youtube.com/watch?v=5HuN9iL-zxU
    IMU->RollRaw = -atan2(MPU6500->fAccelX, sqrt(MPU6500->fAccelY * MPU6500->fAccelY + MPU6500->fAccelZ * MPU6500->fAccelZ)) / (3.142 / 180);
    IMU->PitchRaw = atan2(MPU6500->fAccelY, sqrt(MPU6500->fAccelX * MPU6500->fAccelX + MPU6500->fAccelZ * MPU6500->fAccelZ)) / (3.142 / 180);

    // could not do this but i want to seperate MPU from Kalman filter and let IMU hold all the values  
    IMU->gyroX = MPU6500->fGyroX;
    IMU->GyroY = MPU6500->fGyroY;    
}

void IMU_SetKalmanInput(imu* IMU, kalmanfilter* KalmanFilter) // starting input for kalman filter
{
    KalmanFilter->RollAngle = IMU->RollRaw;
    KalmanFilter->PitchAngle = IMU->PitchRaw;
}

void IMU_GetKalmanOutput(imu* IMU, kalmanfilter* KalmanFilter, float dt) // dt needs to be replaced by loop_time struct 
{   
    // Created by Kristian Lauszus
    // Modified by Filip Michalak
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    /* Step 1 */
    KalmanFilter->RollRate = IMU->gyroX - KalmanFilter->RollBias;
    KalmanFilter->RollAngle += dt* KalmanFilter->RollRate;
    
    KalmanFilter->PitchRate = IMU->GyroY - KalmanFilter->PitchBias;
    KalmanFilter->PitchAngle += dt* KalmanFilter->PitchRate;    

     // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    KalmanFilter->RollP[0][0] += dt * (dt*KalmanFilter->RollP[1][1] - KalmanFilter->RollP[0][1] - KalmanFilter->RollP[1][0] + Q_angle);
    KalmanFilter->RollP[0][1] -= dt * KalmanFilter->RollP[1][1];
    KalmanFilter->RollP[1][0] -= dt * KalmanFilter->RollP[1][1];
    KalmanFilter->RollP[1][1] += Q_bias * dt;

    KalmanFilter->PitchP[0][0] += dt * (dt*KalmanFilter->PitchP[1][1] - KalmanFilter->PitchP[0][1] - KalmanFilter->PitchP[1][0] + Q_angle);
    KalmanFilter->PitchP[0][1] -= dt * KalmanFilter->PitchP[1][1];
    KalmanFilter->PitchP[1][0] -= dt * KalmanFilter->PitchP[1][1];
    KalmanFilter->PitchP[1][1] += Q_bias * dt;

     // Calculate angle and bias - Update estimate with measurement zk (IMU->_Raw)
    /* Step 3 */
    float Ry = IMU->RollRaw - KalmanFilter->RollAngle;  // Roll Angle difference
    
    float Py = IMU->PitchRaw - KalmanFilter->PitchAngle; // Pitch Angle difference
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float Rs = KalmanFilter->RollP[0][0] + R_measure; // Roll Estimate error
    
    float Ps = KalmanFilter->PitchP[0][0] + R_measure; // PitchEstimate error

    /* Step 5 */
    float Rk[2]; // Roll Kalman gain - This is a 2x1 vector
    Rk[0] = KalmanFilter->RollP[0][0] / Rs;
    Rk[1] = KalmanFilter->RollP[1][0] / Rs;

    float Pk[2]; // Pitch Kalman gain - This is a 2x1 vector
    Pk[0] = KalmanFilter->PitchP[0][0] / Ps;
    Pk[1] = KalmanFilter->PitchP[1][0] / Ps;

    /* Step 6 */
    KalmanFilter->RollAngle  += Rk[0] * Ry;
    KalmanFilter->RollBias += Rk[1] * Ry;

    KalmanFilter->PitchAngle  += Pk[0] * Py;
    KalmanFilter->PitchBias += Pk[1] * Py;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float RollP00_temp = KalmanFilter->RollP[0][0];
    float RollP01_temp = KalmanFilter->RollP[0][1];

    KalmanFilter->RollP[0][0] -= Rk[0] * RollP00_temp;
    KalmanFilter->RollP[0][1] -= Rk[0] * RollP01_temp;
    KalmanFilter->RollP[1][0] -= Rk[1] * RollP00_temp;
    KalmanFilter->RollP[1][1] -= Rk[1] * RollP01_temp;

    float PitchP00_temp = KalmanFilter->PitchP[0][0];
    float PitchP01_temp = KalmanFilter->PitchP[0][1];

    KalmanFilter->PitchP[0][0] -= Pk[0] * PitchP00_temp;
    KalmanFilter->PitchP[0][1] -= Pk[0] * PitchP01_temp;
    KalmanFilter->PitchP[1][0] -= Pk[1] * PitchP00_temp;
    KalmanFilter->PitchP[1][1] -= Pk[1] * PitchP01_temp;

    /* Output */
    IMU->RollKal = KalmanFilter->RollAngle;
    IMU->PitchKal = KalmanFilter->PitchAngle;
}

// ---------------------------------------
// Internal functions
// ---------------------------------------