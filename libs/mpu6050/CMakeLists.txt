add_library(mpu6050 INTERFACE)

target_sources(mpu6050 INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/I2Cdev/I2Cdev.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/I2Cdev/I2Cdev.h"
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/MPU6050/MPU6050.h"
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/MPU6050/MPU6050.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/MPU6050/helper_3dmath"
)

target_include_directories(mpu6050 INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/I2Cdev"
    "${CMAKE_CURRENT_LIST_DIR}/../../i2cdevlib/Arduino/MPU6050"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

target_link_libraries(mpu6050 INTERFACE
    arduino-compat
)