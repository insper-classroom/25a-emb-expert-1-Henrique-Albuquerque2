add_executable(main
        main.c
)

target_link_libraries(main pico_stdlib freertos)
pico_add_extra_outputs(main)

target_link_libraries(seu_projeto mpu6050_driver)
