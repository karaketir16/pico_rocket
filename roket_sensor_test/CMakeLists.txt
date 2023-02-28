cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk")

# note: this must happen before project()
include(pico_sdk_import.cmake)

project(roket_sensor_test)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


add_subdirectory( "${CMAKE_CURRENT_LIST_DIR}/../libs/Adafruit_MPU6050"
    build_mpu6050
)
add_subdirectory( "${CMAKE_CURRENT_LIST_DIR}/../libs/Adafruit_BMP3XX"
    build_bmp3xx
)

add_subdirectory( "${CMAKE_CURRENT_LIST_DIR}/../libs/u_blox"
    build_u_blox
)


if (TARGET tinyusb_device)

    add_executable(roket_sensor_transmit
            main_transmit.cpp
            )

    # pull in common dependencies
    target_link_libraries(roket_sensor_transmit 
        pico_stdlib
        pico_multicore
    )

    target_link_libraries(roket_sensor_transmit 
        adafruit-mpu6050
        adafruit-bmp3xx
        hardware_i2c
        u_blox
    )
    

    # enable usb output, disable uart output
    pico_enable_stdio_usb(roket_sensor_transmit 1)
    pico_enable_stdio_uart(roket_sensor_transmit 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(roket_sensor_transmit)




    add_executable(roket_sensor_receive
        main_receive.cpp
    )

    if (NOT TARGET arduino-compat)
        add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
    endif()

    # pull in common dependencies
    target_link_libraries(roket_sensor_receive 
        pico_stdlib
        pico_multicore
        arduino-compat 
    )


    # enable usb output, disable uart output
    pico_enable_stdio_usb(roket_sensor_receive 1)
    pico_enable_stdio_uart(roket_sensor_receive 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(roket_sensor_receive)

    # add url via pico_set_program_url
    # example_auto_set_url(hello_world)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_world because TinyUSB submodule is not initialized in the SDK")
endif()




