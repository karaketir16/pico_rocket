add_library(tiny_gps INTERFACE)

target_sources(tiny_gps INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/TinyGPSPlus/src/TinyGPS++.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/TinyGPSPlus/src/TinyGPS++.h"
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/TinyGPSPlus/src/TinyGPSPlus.h"
)

target_include_directories(tiny_gps INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/TinyGPSPlus/src"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

target_link_libraries(tiny_gps INTERFACE
    arduino-compat
)