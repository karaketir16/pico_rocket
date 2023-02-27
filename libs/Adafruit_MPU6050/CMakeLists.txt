add_library(adafruit-mpu6050 INTERFACE)

target_sources(adafruit-mpu6050 INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_MPU6050/Adafruit_MPU6050.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_MPU6050/Adafruit_MPU6050.h"
)

target_include_directories(adafruit-mpu6050 INTERFACE
 "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_MPU6050"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

if (NOT TARGET adafruit-busio)
    add_subdirectory(../Adafruit_BusIO build_adafruit-busio)
endif()

if (NOT TARGET adafruit-sensor)
    add_subdirectory(../Adafruit_Sensor build_adafruit-sensor)
endif()

target_link_libraries(adafruit-mpu6050 INTERFACE
    arduino-compat
    adafruit-busio
    adafruit-sensor
)
