#ifndef ONESHOT_CONTROL
#define ONESHOT_CONTROL

#include<stdio.h>
#include"pico/stdlib.h"
#include"hardware/pwm.h"

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
    // ONeshot values from 125us - 240us (Note that full range is 250 but we cut last 10 to let the ESC breathe)
    uint16_t fillLF;
    uint16_t fillLB;
    uint16_t fillRF;
    uint16_t fillRB;

}oneshot;

// ---------------------------------------
// Function prototypes
// ---------------------------------------
void initMotors(oneshot* ONESHOT);
void writeMotors(oneshot* ONESHOT);

#endif 