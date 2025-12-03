#include "mpu6500.h"
#include"hardware/clocks.h"

int main()
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    //setup
    mpu6500 MPU6500 = {
        .I2cMPU6500Port = i2c0,
        .MPU6500SclPin = 5, // i2c pin
        .MPU6500SdaPin = 4, // i2c pin
    };
    
    //testing 
    sleep_ms(1000);
    printf("Welcome to testing MPU6500 module.\n");
    sleep_ms(3000);
    MPU6500_I2cInnit(&MPU6500);
    if(!(MPU6500_I2cScanner(&MPU6500)))
    {
        while(1)
        {
            printf("wrong chip! Check pins or module!\n");
            sleep_ms(500);
        }
    }
    printf("Correct module found!");
    MPU6500_Init(&MPU6500);
    while(1)
    {
        MPU6500_ReadData(&MPU6500);
        
        printf("X value %6.3f, Y value  %6.3f, Z value  %6.3f, X gyro value %7.3f, Y gyro value  %7.3f, Z gyro value  %7.3f, The temperature is %5.2f\n",
             MPU6500.fAccelX, MPU6500.fAccelY, MPU6500.fAccelZ, MPU6500.fgyroX, MPU6500.fgyroY, MPU6500.fgyroZ, MPU6500.tempOut);
        sleep_ms(100);
    }
}