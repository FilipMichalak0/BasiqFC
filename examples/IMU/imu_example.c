#include"imu.h"
#include"hardware/clocks.h"

int main()
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    kalmanfilter KalmanFilter;
    imu IMU;
    mpu6500 MPU6500 = {
        .I2cMPU6500Port = i2c0,
        .MPU6500SclPin = 5, // i2c pin
        .MPU6500SdaPin = 4, // i2c pin
    };
    uint32_t timeloop = 4000;
    uint32_t currentTime;
    uint32_t sample_idx = 0;

    uint32_t start;
    uint32_t end;
    uint32_t time;
    MPU6500_Init(&MPU6500);
    IMU_InitializeIMU(&IMU);
    IMU_InitializeKalman(&KalmanFilter);
    MPU6500_ReadData(&MPU6500);
    MPU6500_CalibrateData(&MPU6500);
    IMU_GetInput(&IMU, &MPU6500);
    IMU_SetKalmanInput(&IMU,  &KalmanFilter);
    while(1)
    {
        currentTime = time_us_32();
        MPU6500_ReadData(&MPU6500);
        MPU6500_CalibrateData(&MPU6500);
        IMU_GetInput(&IMU, &MPU6500);
        IMU_GetKalmanOutput(&IMU,  &KalmanFilter, 0.004);
        // printf("Roll value [°] %2.2f | Pitch value [°] %2.2f\n", IMU.RollRaw, IMU.PitchRaw);
        // printf("Roll Kalman Value [°] %2.2f | Pitch Kalman value [°] %2.2f\n", IMU.RollKal, IMU.PitchKal);
        float time_s = sample_idx * 0.004f;

        // Exel format
        printf("%lu;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f; \n",
            (unsigned long)sample_idx,
            time_s,
            IMU.RollRaw,
            IMU.PitchRaw,
            IMU.RollKal, 
            IMU.PitchKal,
            IMU.gyroX,
            IMU.GyroY);

        sample_idx++;
        while(time_us_32() - currentTime < timeloop);
    }
}