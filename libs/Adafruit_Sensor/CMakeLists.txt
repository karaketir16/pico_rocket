add_library(adafruit-sensor INTERFACE)

target_sources(adafruit-sensor INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_Sensor/Adafruit_Sensor.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_Sensor/Adafruit_Sensor.h"
)

target_include_directories(adafruit-sensor INTERFACE
 "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_Sensor"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

if (NOT TARGET adafruit-busio)
    add_subdirectory(../Adafruit_BusIO build_adafruit-busio)
endif()

target_link_libraries(adafruit-sensor INTERFACE
    arduino-compat
    adafruit-busio
)
