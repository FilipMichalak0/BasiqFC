#ifndef ONESHOT_CONTROL_
#define ONESHOT_CONTROL_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "mma.h"

// ---------------------------------------
// Oneshot125 struct
// ---------------------------------------
typedef struct 
{
    // gpio values for initialization
    uint8_t motorLF; // left front 
    uint8_t motorLB; // left back
    uint8_t motorRF; // right front 
    uint8_t motorRB; // right back
    // GPIO slices for PWM 
    uint8_t slice_num_LF;
    uint8_t slice_num_LB;
    uint8_t slice_num_RF; 
    uint8_t slice_num_RB;
    // Oneshot values from 125us - 240us (Note that full range is 250 but we cut last 10 to let the ESC breathe)
    uint16_t fillLF;
    uint16_t fillLB;
    uint16_t fillRF;
    uint16_t fillRB;

}oneshot;

// ---------------------------------------
// Public API
// ---------------------------------------
void ONESHOT_initMotors(oneshot* Oneshot);
void ONESHOT_writeMotors(oneshot* Oneshot);
void ONESHOT_CalculateOutput(oneshot* Oneshot, mma* MMA);

#endif // ONESHOT_CONTROL_