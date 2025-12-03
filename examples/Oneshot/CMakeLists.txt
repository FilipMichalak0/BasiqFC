add_executable(BASIQ_oneshot_example
    oneshot_example.c          
    ${CMAKE_SOURCE_DIR}/src/oneshot.c  
    ${CMAKE_SOURCE_DIR}/src/crsf.c
)
target_include_directories(BASIQ_oneshot_example PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_oneshot_example "BASIQ_oneshot_example")
pico_set_program_version(BASIQ_oneshot_example "0.1")

pico_enable_stdio_uart(BASIQ_oneshot_example 0)
pico_enable_stdio_usb(BASIQ_oneshot_example 1)

target_link_libraries(BASIQ_oneshot_example
        pico_stdlib
        hardware_pwm
)

pico_add_extra_outputs(BASIQ_oneshot_example)