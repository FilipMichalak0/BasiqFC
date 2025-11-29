# Add executable. Default name is the project name, version 0.1

add_executable(BASIC_quadcopter main.c )

pico_set_program_name(BASIC_quadcopter "BASIC_quadcopter")
pico_set_program_version(BASIC_quadcopter "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(BASIC_quadcopter 0)
pico_enable_stdio_usb(BASIC_quadcopter 0)

# Add the standard library to the build
target_link_libraries(BASIC_quadcopter
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(BASIC_quadcopter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(BASIC_quadcopter)