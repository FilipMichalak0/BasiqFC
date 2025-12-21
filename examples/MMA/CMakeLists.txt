add_executable(BASIQ_mma_example
    mma_example.c 
    ${CMAKE_SOURCE_DIR}/src/mma.c 
)

target_include_directories(BASIQ_mma_example PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(BASIQ_mma_example "BASIQ_mma_example")
pico_set_program_version(BASIQ_mma_example "0.1")

pico_enable_stdio_uart(BASIQ_mma_example 0)
pico_enable_stdio_usb(BASIQ_mma_example 1)

target_link_libraries(BASIQ_mma_example
        pico_stdlib
)

pico_add_extra_outputs(BASIQ_mma_example)