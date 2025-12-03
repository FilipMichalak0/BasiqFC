#include"crsf.h"
#include<stdio.h>
#include"hardware/clocks.h"

int main(void)
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    crsf_data crsf = 
    {
        .UartCRSFPort = uart0,
        .UartTxPin = 0,
        .UartRxPin = 1
        
    };
    CRSF_UartInit(&crsf);
    CRSF_Init(&crsf);
    while(1)
    {
        CRSF_StateMachine(&crsf);
        CRSF_ChannelMaping(&crsf);
        printf("ch1 = %d ", crsf.PWMData[0]);
        printf("ch2 = %d ", crsf.PWMData[1]);
        printf("ch3 = %d ", crsf.PWMData[2]);
        printf("ch4 = %d  \n", crsf.PWMData[3]);
        
        sleep_ms(5);
    }
    return 0;
}