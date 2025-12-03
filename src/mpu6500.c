/*
Filip Michalak 2025

BME280 Driver for Drone project 

This driver is only suitable for drone application and it should be only used for this purpose

This driver assumes you are using only one BME280 module as it wasnt tested with multiple ones
*/

#include "mpu6500.h"
// ---------------------------------------
// Functions 
// ---------------------------------------
uint8_t MPU6500_I2cScanner(mpu6500* MPU6500) // scanning if correct module is being used
{
    sleep_ms(1000); // waiting for I2C to set up 
    uint8_t chipID[1];
    uint8_t reg = 0x75;
    i2c_write_blocking(MPU6500->I2cMPU6500Port, MPU6500_address, &reg, 1, true);
    i2c_read_blocking(MPU6500->I2cMPU6500Port, MPU6500_address, &chipID[0], 1, false);
    printf("The chip's ID is 0x%X",chipID[0]); 
    if(chipID[0] == 0x70) 
    {
        return 1;
    }
    return 0;
}
static void MPU6500_writeSingleData(uint8_t reg, uint8_t value, mpu6500* MPU6500) // sending single data 
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(MPU6500->I2cMPU6500Port, MPU6500_address, data, 2, false);
} 
void MPU6500_Init(mpu6500* MPU6500) // initialization of MPU6500 module 
{
    MPU6500_writeSingleData(0x6B, 0x01, MPU6500); // waking up the MPU and setting internal oscilator to best available
    MPU6500_writeSingleData(0x6C, 0x00, MPU6500); // ensuring that all axis are working 
    MPU6500_writeSingleData(0x1A, 0x03, MPU6500); // DLPF 
    MPU6500_writeSingleData(0x1B, 0x08, MPU6500); // range +/-500 dps
    MPU6500_writeSingleData(0x1C, 0x00, MPU6500); // range =/- 2g
    MPU6500_writeSingleData(0x1D, 0x03, MPU6500); // DLPF 
    MPU6500_writeSingleData(0x19, 0x09, MPU6500); // divider if DLPF
}
void MPU6500_I2cInnit(mpu6500* MPU6500) // initialiaze MPU6500 i2c communication protocol 
{
    i2c_init(MPU6500->I2cMPU6500Port, 400 * 1000);
    gpio_set_function(MPU6500->MPU6500SclPin, GPIO_FUNC_I2C);
    gpio_set_function(MPU6500->MPU6500SdaPin, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6500->MPU6500SclPin);
    gpio_pull_up(MPU6500->MPU6500SdaPin);
    MPU6500->val = 0x3B;
}
void MPU6500_ReadData(mpu6500* MPU6500) // reading all the needed data and storing in MPU6500 struct 
{
        i2c_write_blocking(MPU6500->I2cMPU6500Port, MPU6500_address, &MPU6500->val, 1, true);
        i2c_read_blocking(MPU6500->I2cMPU6500Port, MPU6500_address, MPU6500->buff, 14, false);

        MPU6500->accelX = ((MPU6500->buff[0]<<8) | MPU6500->buff[1]);
        MPU6500->accelY = ((MPU6500->buff[2]<<8) | MPU6500->buff[3]);
        MPU6500->accelZ = ((MPU6500->buff[4]<<8) | MPU6500->buff[5]);

        MPU6500->temperature = ((MPU6500->buff[6] << 8) | MPU6500->buff[7]);

        MPU6500->tempOut = (MPU6500->temperature / 333.87f) + 21.0f;
        MPU6500->gyroX = ((MPU6500->buff[8]<<8) | MPU6500->buff[9]);
        MPU6500->gyroY = ((MPU6500->buff[10]<<8) | MPU6500->buff[11]);
        MPU6500->gyroZ = ((MPU6500->buff[12]<<8) | MPU6500->buff[13]);
        
        MPU6500->fAccelX = MPU6500->accelX / 16384.0f;
        MPU6500->fAccelY = MPU6500->accelY / 16384.0f;
        MPU6500->fAccelZ = MPU6500->accelZ / 16384.0f;

        MPU6500->fgyroX = MPU6500->gyroX / 65.5f;
        MPU6500->fgyroY = MPU6500->gyroY / 65.5f;
        MPU6500->fgyroZ = MPU6500->gyroZ / 65.5f;
}