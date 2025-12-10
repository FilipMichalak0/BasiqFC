target_include_directories(BASIQ_quadcopter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
add_subdirectory(Oneshot)
add_subdirectory(MPU6500)
add_subdirectory(BME280)
add_subdirectory(CRSF)
add_subdirectory(IMU)
add_subdirectory(Loop_time)
add_subdirectory(PID_controller)