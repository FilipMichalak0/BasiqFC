add_executable(BASIQ_mpu6500_example 
    mpu6500_example.c 
    ${CMAKE_SOURCE_DIR}/src/mpu6500.c 
)

target_include_directories(BASIQ_mpu6500_example PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_mpu6500_example "BASIQ_mpu6500_example")
pico_set_program_version(BASIQ_mpu6500_example "0.1")

pico_enable_stdio_uart(BASIQ_mpu6500_example 0)
pico_enable_stdio_usb(BASIQ_mpu6500_example 1)

target_link_libraries(BASIQ_mpu6500_example
        pico_stdlib
        hardware_i2c
)

pico_add_extra_outputs(BASIQ_mpu6500_example)