#include"oneshot.h"

void initMotors(oneshot* ONESHOT) // initialization of all PWM channels needed for all the motors
{
    gpio_set_function(ONESHOT->motorLF, GPIO_FUNC_PWM);
    gpio_set_function(ONESHOT->motorLB, GPIO_FUNC_PWM);
    gpio_set_function(ONESHOT->motorRF, GPIO_FUNC_PWM);
    gpio_set_function(ONESHOT->motorRB, GPIO_FUNC_PWM);
    
    // getting the correct slice for each motor that enables basic functionality of PWM 
    ONESHOT->slice_num_LF = pwm_gpio_to_slice_num(ONESHOT->motorLF);
    ONESHOT->slice_num_LB = pwm_gpio_to_slice_num(ONESHOT->motorLB); 
    ONESHOT->slice_num_RF = pwm_gpio_to_slice_num(ONESHOT->motorRF);
    ONESHOT->slice_num_RB = pwm_gpio_to_slice_num(ONESHOT->motorRB);

    pwm_config config = pwm_get_default_config();
    // setting up the divider for correct HZ (aprox. 4kHz) 
    pwm_config_set_clkdiv_int(&config,220);

    pwm_init(ONESHOT->slice_num_LF, &config,true);
    pwm_init(ONESHOT->slice_num_LB, &config,true);
    pwm_init(ONESHOT->slice_num_RF, &config,true);
    pwm_init(ONESHOT->slice_num_RB, &config,true);
    // max value after wich it will wrap around and count again
    pwm_set_wrap(ONESHOT->slice_num_LF, 249);
    pwm_set_wrap(ONESHOT->slice_num_LB, 249);
    pwm_set_wrap(ONESHOT->slice_num_RF, 249);
    pwm_set_wrap(ONESHOT->slice_num_RB, 249);

    pwm_set_chan_level(ONESHOT->slice_num_LF, PWM_CHAN_A, 125);
    pwm_set_chan_level(ONESHOT->slice_num_LB, PWM_CHAN_A, 125);
    pwm_set_chan_level(ONESHOT->slice_num_RF, PWM_CHAN_A, 125);
    pwm_set_chan_level(ONESHOT->slice_num_RB, PWM_CHAN_A, 125);

    pwm_set_enabled(ONESHOT->slice_num_LF, true);
    pwm_set_enabled(ONESHOT->slice_num_LB, true);
    pwm_set_enabled(ONESHOT->slice_num_RF, true);
    pwm_set_enabled(ONESHOT->slice_num_RB, true);
}

void writeMotors(oneshot* ONESHOT)
{
    pwm_set_chan_level(ONESHOT->slice_num_LF, PWM_CHAN_A, ONESHOT->fillLF);
    pwm_set_chan_level(ONESHOT->slice_num_LB, PWM_CHAN_B, ONESHOT->fillLB);
    pwm_set_chan_level(ONESHOT->slice_num_RF, PWM_CHAN_A, ONESHOT->fillRF);
    pwm_set_chan_level(ONESHOT->slice_num_RB, PWM_CHAN_B, ONESHOT->fillRB);
}