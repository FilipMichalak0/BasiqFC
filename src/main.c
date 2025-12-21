#include <stdio.h>
#include "pico/stdlib.h"
#include "crsf.h"
#include "imu.h"
#include "input_control.h"
#include "loop_time.h"
#include "mma.h"
#include "mpu6500.h"
#include "oneshot.h"
#include "pid.h"

enum state  
{
    NotArmed,
    Armed 
};

int main()
{
    // loopTime struct 
    
    loop_time LOOP_TIME =
    {
        .sysSpeed = 220000000,
        .loopSpeed = 4000,
        .dt = 0.004f
    };
    // overclocking raspberry pi pico
    setSystemClockSpeed(&LOOP_TIME);
    stdio_init_all();

    enum state State = NotArmed;
    // MPU6500 struct
    mpu6500 MPU6500 = 
    {
        .I2cMPU6500Port = i2c1,
        .MPU6500SclPin = 3,
        .MPU6500SdaPin = 2
    }; 

    // CRSF struct 
    crsf_data CRSF = 
    {
        .UartCRSFPort = uart0,
        .UartTxPin = 12,
        .UartRxPin = 13
    };

    // Oneshot struct
    oneshot ONESHOT =
    {
        .motorLF = 14,
        .motorLB = 15,
        .motorRF = 17,
        .motorRB = 16,
        .fillLB = 125,
        .fillLF = 125,
        .fillRB = 125,
        .fillRF = 125
    };

    // IMU struct 
    imu IMU;
    angleKalmanFilter RollKalmanFilter;
    angleKalmanFilter PitchKalmanFilter;

    // input controll struct
    input_control INPUT_CONTROL = {0, 0, 0, 0};

    // PID structs for Roll, Pitch, Yaw 
    pid PIDRoll = 
    {
        .Kp = 1,
        .Ki = 0.001,
        .Kd = 0.1,
        .antyWindupMax = 300,
        .antyWindupMin = -300,
        .dt = 0.004f
    };
    pid PIDPitch = 
    {
        .Kp = 2,
        .Ki = 0.001,
        .Kd = 0.1,
        .antyWindupMax = 300,
        .antyWindupMin = -300,
        .dt = 0.004f
    };
    pid PIDYaw = 
    {
        .Kp = 2,
        .Ki = 1,
        .Kd = 0.001,
        .antyWindupMax = 300,
        .antyWindupMin = -300,
        .dt = 0.004f
    };

    // mma struct 
    mma MMA;


    // initialization of MPU6500  
    MPU6500_Init(&MPU6500);

    // initialization of CRSF protocol 
    CRSF_Init(&CRSF);

    // initialization of Oneshot protocol 
    ONESHOT_initMotors(&ONESHOT);

    // initialization of imu
    IMU_InitializeIMU(&IMU);

    IMU_AngleInitializeKalman(&RollKalmanFilter);
    IMU_AngleInitializeKalman(&PitchKalmanFilter);
    
    // initialization of pid
    PID_init(&PIDRoll);
    PID_init(&PIDPitch);
    PID_init(&PIDYaw); 

    // Calibration of all modules 
    // MPU6500
    MPU6500_ReadData(&MPU6500);
    MPU6500_CalibrateData(&MPU6500);

    // IMU
    IMU_AngleGetInput(&IMU, &MPU6500);
    // Kalmans 
    IMU_AngleSetKalmanInput(IMU.RollRaw,  &RollKalmanFilter);
    IMU_AngleSetKalmanInput(IMU.PitchRaw,  &PitchKalmanFilter);
    // maybe put it in some other file to manage it 
    printSystemClockSpeed(&LOOP_TIME);
    
    // while loop 
    while(1)
    {   
        // start of main loop control 
        startLoop(&LOOP_TIME);
        
        // read IMU data
        MPU6500_ReadData(&MPU6500);
        MPU6500_CalibrateData(&MPU6500);

        IMU_AngleGetInput(&IMU, &MPU6500);

        IMU.RollKal =  IMU_AngleGetKalmanOutput(IMU.GyroX, IMU.RollRaw,  &RollKalmanFilter, LOOP_TIME.dt);
        IMU.PitchKal =  IMU_AngleGetKalmanOutput(IMU.GyroY, IMU.PitchRaw,  &PitchKalmanFilter, LOOP_TIME.dt);

        // printf("IMU ROLL = %.4f, IMU PITCH = %.4f\n", IMU.RollKal, IMU.PitchKal);
        // read transmiter data
        CRSF_StateMachine(&CRSF);
        INPUT_CONTROL_CalculateInput(&INPUT_CONTROL, &CRSF);
        // printf("Input throttle = %f", INPUT_CONTROL.throttle);
        // check if we are ready to fly 
        if(INPUT_CONTROL_isArmed(&CRSF) && State == NotArmed && INPUT_CONTROL.throttle < 1050)
        {
            State = Armed;
        }
        // check if we want to stop flying 
        if(!(INPUT_CONTROL_isArmed(&CRSF)))
        {
            State = NotArmed;
            
        }
        
        if(State == NotArmed)
        {
            // Reset all earlier values for again takeoff
            PID_reset(&PIDRoll);
            PID_reset(&PIDPitch);
            PID_reset(&PIDYaw);
            ONESHOT.fillLB = 125;
            ONESHOT.fillLF = 125;
            ONESHOT.fillRB = 125;
            ONESHOT.fillRF = 125;
            ONESHOT_writeMotors(&ONESHOT);

            //printf("State not Armed!");
            continue;
        }
        //printf("State Armed!");
        // PID calculations 
        PID_calculate(&PIDRoll, INPUT_CONTROL.roll, IMU.RollKal);
        PID_calculate(&PIDPitch, INPUT_CONTROL.pitch, IMU.PitchKal);
        PID_calculate(&PIDYaw, INPUT_CONTROL.yaw, IMU.YawRaw);

        // limit throttle for other calculations to balance quadcopter
        INPUT_CONTROL_LimitThrottle(&INPUT_CONTROL);
        // caluclations for mototrs
        MMA_calculateOutput(&MMA, PIDRoll.output, PIDPitch.output, INPUT_CONTROL.throttle, PIDYaw.output);
        printf("PID Roll = %f, PID Pitch = %f, Input Throttle = %f, PID Yaw = %f\n", PIDRoll.output, PIDPitch.output, INPUT_CONTROL.throttle, PIDYaw.output);
        MMA_LimitOutput(&MMA);
        ONESHOT_CalculateOutput(&ONESHOT, &MMA);
        ONESHOT_writeMotors(&ONESHOT); 
        // 
        // end of loop to match 250Hz

        endLoop(&LOOP_TIME);
    }
    // get input from mpu6500 
    // IMU calculation 
    // CRSF protocol reading 
    // Input control
    // Ask we are armed if not we want to reset earlier values 
    // PID calculations 
    // mma calculations 
    // oneshot protocol to power motors 
    // check how much time have passed to see if are good with timings 
    // wait for loop time to reach 250hz 
    

    return 0;
}
