# Add executable. Default name is the project name, version 0.1

add_executable(BASIQ_quadcopter main.c crsf.c imu.c input_control.c loop_time.c mma.c mpu6500.c  oneshot.c pid.c)

pico_set_program_name(BASIQ_quadcopter "BASIQ_quadcopter")
pico_set_program_version(BASIQ_quadcopter "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(BASIQ_quadcopter 0)
pico_enable_stdio_usb(BASIQ_quadcopter 1)

# Add the standard library to the build
target_link_libraries(BASIQ_quadcopter
        pico_stdlib
        hardware_pwm
        hardware_clocks
        hardware_i2c
        )

# Add the standard include files to the build
target_include_directories(BASIQ_quadcopter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(BASIQ_quadcopter)