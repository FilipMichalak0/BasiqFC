add_executable(BASIQ_pid_controller_example 
    pid_controller_example.c 
    ${CMAKE_SOURCE_DIR}/src/pid.c 
)

target_include_directories(BASIQ_pid_controller_example PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_pid_controller_example "BASIQ_pid_controller_example")
pico_set_program_version(BASIQ_pid_controller_example "0.1")

pico_enable_stdio_uart(BASIQ_pid_controller_example 0)
pico_enable_stdio_usb(BASIQ_pid_controller_example 1)

target_link_libraries(BASIQ_pid_controller_example
        pico_stdlib
        hardware_i2c
)

pico_add_extra_outputs(BASIQ_pid_controller_example)