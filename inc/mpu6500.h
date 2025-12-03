#ifndef MPU6500_DRIVER
#define MPU6500_DRIVER

#include<stdio.h>
#include"pico/stdlib.h"
#include"hardware/i2c.h"

// ---------------------------------------
// I2C setup
// ---------------------------------------
#define MPU6500_address 0x68

// ---------------------------------------
// MPU6500 structs
// ---------------------------------------
typedef struct
{
    i2c_inst_t* I2cMPU6500Port;
    uint8_t MPU6500SclPin;
    uint8_t MPU6500SdaPin;
    uint8_t val;
    uint8_t buff[14];
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temperature;
    float tempOut;
    float fAccelX, fAccelY, fAccelZ;
    float fgyroX, fgyroY, fgyroZ;
    
}mpu6500;

// ---------------------------------------
// Prototypes
// ---------------------------------------
uint8_t MPU6500_I2cScanner(mpu6500* MPU6500);
static void MPU6500_writeSingleData(uint8_t reg, uint8_t value,mpu6500* MPU6500);
void MPU6500_Init(mpu6500* MPU6500);
void MPU6500_I2cInnit(mpu6500* MPU6500);
void MPU6500_ReadData(mpu6500* MPU6500);

#endif