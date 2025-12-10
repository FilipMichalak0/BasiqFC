add_executable(BASIQ_loop_time_example 
    Loop_time_example.c 
    ${CMAKE_SOURCE_DIR}/src/loop_time.c 
)

target_include_directories(BASIQ_loop_time_example PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_loop_time_example "BASIQ_loop_time_example")
pico_set_program_version(BASIQ_loop_time_example "0.1")

pico_enable_stdio_uart(BASIQ_loop_time_example 0)
pico_enable_stdio_usb(BASIQ_loop_time_example 1)

target_link_libraries(BASIQ_loop_time_example
        pico_stdlib
        hardware_clocks
)

pico_add_extra_outputs(BASIQ_loop_time_example)