#include "oneshot.h"

void ONESHOT_initMotors(oneshot* Oneshot) // initialization of all PWM channels needed for all the motors
{
    gpio_set_function(Oneshot->motorLF, GPIO_FUNC_PWM);
    gpio_set_function(Oneshot->motorLB, GPIO_FUNC_PWM);
    gpio_set_function(Oneshot->motorRF, GPIO_FUNC_PWM);
    gpio_set_function(Oneshot->motorRB, GPIO_FUNC_PWM);
    
    // getting the correct slice for each motor that enables basic functionality of PWM 
    Oneshot->slice_num_LF = pwm_gpio_to_slice_num(Oneshot->motorLF);
    Oneshot->slice_num_LB = pwm_gpio_to_slice_num(Oneshot->motorLB); 
    Oneshot->slice_num_RF = pwm_gpio_to_slice_num(Oneshot->motorRF);
    Oneshot->slice_num_RB = pwm_gpio_to_slice_num(Oneshot->motorRB);

    pwm_config config = pwm_get_default_config();
    // setting up the divider for correct HZ (aprox. 4kHz) 
    pwm_config_set_clkdiv_int(&config,220);

    pwm_init(Oneshot->slice_num_LF, &config,true);
    pwm_init(Oneshot->slice_num_LB, &config,true);
    pwm_init(Oneshot->slice_num_RF, &config,true);
    pwm_init(Oneshot->slice_num_RB, &config,true);
    // max value after wich it will wrap around and count again
    pwm_set_wrap(Oneshot->slice_num_LF, 249);
    pwm_set_wrap(Oneshot->slice_num_LB, 249);
    pwm_set_wrap(Oneshot->slice_num_RF, 249);
    pwm_set_wrap(Oneshot->slice_num_RB, 249);

    pwm_set_chan_level(Oneshot->slice_num_LF, PWM_CHAN_A, 125);
    pwm_set_chan_level(Oneshot->slice_num_LB, PWM_CHAN_B, 125);
    pwm_set_chan_level(Oneshot->slice_num_RF, PWM_CHAN_B, 125);
    pwm_set_chan_level(Oneshot->slice_num_RB, PWM_CHAN_A, 125);

    pwm_set_enabled(Oneshot->slice_num_LF, true);
    pwm_set_enabled(Oneshot->slice_num_LB, true);
    pwm_set_enabled(Oneshot->slice_num_RF, true);
    pwm_set_enabled(Oneshot->slice_num_RB, true);
}

void ONESHOT_writeMotors(oneshot* Oneshot) 
{
    pwm_set_chan_level(Oneshot->slice_num_LF, PWM_CHAN_A, Oneshot->fillLF);
    pwm_set_chan_level(Oneshot->slice_num_LB, PWM_CHAN_B, Oneshot->fillLB);
    pwm_set_chan_level(Oneshot->slice_num_RF, PWM_CHAN_B, Oneshot->fillRF);
    pwm_set_chan_level(Oneshot->slice_num_RB, PWM_CHAN_A, Oneshot->fillRB);
}

void ONESHOT_CalculateOutput(oneshot* Oneshot, mma* MMA)
{
    Oneshot->fillLF = (MMA->motorLF * 125) / 1000;
    Oneshot->fillLB = (MMA->motorLB * 125) / 1000;
    Oneshot->fillRF = (MMA->motorRF * 125) / 1000;
    Oneshot->fillRB = (MMA->motorRB * 125) / 1000;
}