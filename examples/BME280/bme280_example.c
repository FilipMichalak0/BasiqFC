#include"bme280.h"
#include"hardware/clocks.h"
// ---------------------------------------
// Main function 
// ---------------------------------------
int main(void)
{
    //setup
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    bme280 BME280 = 
    {
        .I2cBME280Port = i2c0,
        .BME280SdaPin = 4,
        .BME280SclPin = 5 
    };
    BME280_I2cInnit(&BME280);
    sleep_ms(3000);
    printf("Welcome!\n");
    sleep_ms(1000);
    printf("Clock speed is set to %d", clock_get_hz(clk_sys));
    if(!(BME280_i2cScanner(&BME280)))
    {
        while(1)
        {
            printf("Wrong chip! Please check module or wiring!");
            sleep_ms(500);
        }
    }
    printf("Correct chip detected\n");
    BME280_Init(&BME280);
    BME280_readCalibrationData(&BME280);
    sleep_ms(100);
    BME280.P0Pa = BME280_CalculateReference(200,&BME280);

    //loop
    while(1)
    {
        BME280_ReadData(&BME280);
        BME280_CalculateAltitude(&BME280);
        printf("P = %lu.%02lu hPa | Alt rel = %ld.%02ld m\n",
            BME280.hPa / 100, BME280.hPa % 100,BME280.altitudeCM / 100, BME280.altitudeCM % 100);

        sleep_ms(200); 
    }

    return 0;
}