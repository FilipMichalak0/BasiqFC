#include "loop_time.h"
int main()
{
    loop_time LoopTime =
    {
        .sysSpeed = 220000000,
        .loopSpeed = 4000,
        .dt = 0.004f
    };
    setSystemClockSpeed(&LoopTime);
    stdio_init_all();
    
    while(1)
    {
        startLoop(&LoopTime);
        printSystemClockSpeed(&LoopTime);
        checkLoop(&LoopTime);
        endLoop(&LoopTime);
        sleep_ms(100); // should't be used here but it is here so that the serial is not bloated with ton of prints
    }
}