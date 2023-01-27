cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_mpu6050)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


add_subdirectory( "${CMAKE_CURRENT_LIST_DIR}/../../libs/mpu6050"
    build_mpu6050
)
add_subdirectory( "${CMAKE_CURRENT_LIST_DIR}/../../libs/Adafruit_BMP3XX"
    build_bmp3xx
)

# add_subdirectory(/home/karaketir16/proj/pico-arduino-compat/libs/adafruit-bmp3xx-library/
#     build_bmp3xx
# )
# rest of your project


if (TARGET tinyusb_device)
    add_executable(pico_mpu6050
            main.cpp
            i2c_test.cpp
            i2cScan.cpp
            )

    # pull in common dependencies
    target_link_libraries(pico_mpu6050 
        pico_stdlib
        pico_multicore
    )
    

    # enable usb output, disable uart output
    pico_enable_stdio_usb(pico_mpu6050 1)
    pico_enable_stdio_uart(pico_mpu6050 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(pico_mpu6050)

    # add url via pico_set_program_url
    # example_auto_set_url(hello_world)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_world because TinyUSB submodule is not initialized in the SDK")
endif()


target_link_libraries(pico_mpu6050 
    mpu6050
    adafruit-bmp3xx
    hardware_i2c
)