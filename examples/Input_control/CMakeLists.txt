add_executable(BASIQ_input_control_example
    input_control_example.c 
    ${CMAKE_SOURCE_DIR}/src/input_control.c 
    ${CMAKE_SOURCE_DIR}/src/crsf.c 
)

target_include_directories(BASIQ_input_control_example PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_input_control_example "BASIQ_input_control_example")
pico_set_program_version(BASIQ_input_control_example "0.1")

pico_enable_stdio_uart(BASIQ_input_control_example 0)
pico_enable_stdio_usb(BASIQ_input_control_example 1)

target_link_libraries(BASIQ_input_control_example
        pico_stdlib
)

pico_add_extra_outputs(BASIQ_input_control_example)