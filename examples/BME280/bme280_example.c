#include"bme280.h"
#include"hardware/clocks.h"
// ---------------------------------------
// Main function 
// ---------------------------------------
int main(void)
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    bme280 BME280 = 
    {
        .I2cBME280Port = i2c0,
        .BME280SdaPin = 4,
        .BME280SclPin = 5 
    };
    BME280_Init(&BME280);
    while(1)
    {
        BME280_ReadData(&BME280);
        printf("P = %lu.%02lu hPa \n",
            BME280.hPa / 100, BME280.hPa % 100);
        sleep_ms(200); 
    }
    return 0;
}