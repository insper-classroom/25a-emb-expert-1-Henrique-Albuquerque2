add_library(mpu6050_driver 
    mpu6050.c
    mpu6050.h
)

target_include_directories(mpu6050_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(mpu6050_driver
    hardware_i2c
    pico_stdlib
)
