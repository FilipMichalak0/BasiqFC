#include "mma.h"

void MMA_calculateOutput(mma* MMA, float roll, float pitch, float throttle, float yaw)
{
    MMA->motorRF = throttle - roll - pitch - yaw;
    MMA->motorRB = throttle - roll + pitch + yaw;
    MMA->motorLB = throttle + roll + pitch - yaw;
    MMA->motorLF = throttle + roll - pitch + yaw;

}

void MMA_LimitOutput(mma* MMA)
{
    // not allowing for motors to get to much power 
    if(MMA->motorRF > 1900) MMA->motorRF = 1900; 
    if(MMA->motorRB > 1900) MMA->motorRB = 1900; 
    if(MMA->motorLB > 1900) MMA->motorLB = 1900; 
    if(MMA->motorLF > 1900) MMA->motorLF = 1900;
    
    // not alowing for motors to go down 
    if(MMA->motorRF < 1050) MMA->motorRF = 1050; 
    if(MMA->motorRB < 1050) MMA->motorRB = 1050; 
    if(MMA->motorLB < 1050) MMA->motorLB = 1050; 
    if(MMA->motorLF < 1050) MMA->motorLF = 1050;
}

