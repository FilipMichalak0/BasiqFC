add_executable(BASIQ_crsf_example 
        crsf_example.c 
        ${CMAKE_SOURCE_DIR}/src/crsf.c
)

target_include_directories(BASIQ_crsf_example PRIVATE
        ${CMAKE_SOURCE_DIR}/inc
        ${CMAKE_CURRENT_LIST_DIR}        
)

pico_set_program_name(BASIQ_crsf_example "BASIQ_crsf_example")
pico_set_program_version(BASIQ_crsf_example "0.1")

pico_enable_stdio_uart(BASIQ_crsf_example 0)
pico_enable_stdio_usb(BASIQ_crsf_example 1)

target_link_libraries(BASIQ_crsf_example
        pico_stdlib
)

pico_add_extra_outputs(BASIQ_crsf_example)

