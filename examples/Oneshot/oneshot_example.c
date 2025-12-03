#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "oneshot.h"
#include "crsf.h"

int main()
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    crsf_data Crsf = {
        .UartCRSFPort = uart0,
        .UartTxPin = 0,
        .UartRxPin = 1
    };
    CRSF_UartInit(&Crsf);
    CRSF_Init(&Crsf);
    oneshot Oneshot = {
        .motorLF = 10,
        .motorLB = 11,
        .motorRF = 12,
        .motorRB = 13
    };
    initMotors(&Oneshot);

    while (true) 
    {
        CRSF_StateMachine(&Crsf);
        CRSF_ChannelMaping(&Crsf);
        /*
        printf("ch1 = %d ", Crsf.PWMData[0]);
        printf("ch2 = %d ", Crsf.PWMData[1]);
        printf("ch3 = %d ", Crsf.PWMData[2]);
        printf("ch4 = %d  \n", Crsf.PWMData[3]);
        */
        Oneshot.fillLF = (Crsf.PWMData[0] * 125) / 1000;
        if(Oneshot.fillLF > 240) Oneshot.fillLF = 240;
        Oneshot.fillLB = (Crsf.PWMData[1] * 125) / 1000;
        if(Oneshot.fillLB > 240) Oneshot.fillLB = 240;
        Oneshot.fillRF = (Crsf.PWMData[2] * 125) / 1000;
        if(Oneshot.fillRF > 240) Oneshot.fillRF = 240;
        Oneshot.fillRB = (Crsf.PWMData[3] * 125) / 1000;
        if(Oneshot.fillRB > 240) Oneshot.fillRB = 240;
        printf("LF = %d ", Oneshot.fillLF);
        printf("LB = %d ", Oneshot.fillLB);
        printf("RF = %d ", Oneshot.fillRF);
        printf("RB = %d  \n", Oneshot.fillRB);
        writeMotors(&Oneshot);
        sleep_ms(5);
    }
}