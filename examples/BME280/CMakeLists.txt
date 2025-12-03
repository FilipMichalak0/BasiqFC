add_executable(BASIQ_bme280_example 
        bme280_example.c 
        ${CMAKE_SOURCE_DIR}/src/bme280.c
)

target_include_directories(BASIQ_bme280_example PRIVATE
        ${CMAKE_SOURCE_DIR}/inc
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_bme280_example "BASIQ_bme280_example")
pico_set_program_version(BASIQ_bme280_example "0.1")

pico_enable_stdio_uart(BASIQ_bme280_example 0)
pico_enable_stdio_usb(BASIQ_bme280_example 1)

target_link_libraries(BASIQ_bme280_example
        pico_stdlib
        hardware_i2c
)   

pico_add_extra_outputs(BASIQ_bme280_example)

