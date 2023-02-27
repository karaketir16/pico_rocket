add_library(adafruit-bmp3xx INTERFACE)

target_sources(adafruit-bmp3xx INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_BMP3XX/Adafruit_BMP3XX.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_BMP3XX/bmp3.c"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_BMP3XX/bmp3_defs.h"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_BMP3XX/bmp3.h"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_BMP3XX/Adafruit_BMP3XX.h"
)

target_include_directories(adafruit-bmp3xx INTERFACE
 "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/Adafruit_BMP3XX"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

if (NOT TARGET adafruit-busio)
    add_subdirectory(../Adafruit_BusIO build_adafruit-busio)
endif()

target_link_libraries(adafruit-bmp3xx INTERFACE
    arduino-compat
    adafruit-busio
)
