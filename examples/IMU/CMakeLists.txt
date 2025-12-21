add_executable(BASIQ_imu_example
        imu_example.c
        ${CMAKE_SOURCE_DIR}/src/imu.c
        ${CMAKE_SOURCE_DIR}/src/mpu6500.c
        ${CMAKE_SOURCE_DIR}/src/bme280.c
)

target_include_directories(BASIQ_imu_example PRIVATE
        ${CMAKE_SOURCE_DIR}/inc  
        ${CMAKE_CURRENT_LIST_DIR}      
)

pico_set_program_name(BASIQ_imu_example "BASIQ_imu_example")
pico_set_program_version(BASIQ_imu_example "0.1")

pico_enable_stdio_usb(BASIQ_imu_example 1)
pico_enable_stdio_uart(BASIQ_imu_example 0)

target_link_libraries(BASIQ_imu_example
        pico_stdlib
        hardware_i2c
        hardware_clocks
        )

pico_add_extra_outputs(BASIQ_imu_example)