add_library(u_blox INTERFACE)

target_sources(u_blox INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/SparkFun_u-blox_GNSS_Arduino_Library/src/SparkFun_u-blox_GNSS_Arduino_Library.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/SparkFun_u-blox_GNSS_Arduino_Library/src/SparkFun_u-blox_GNSS_Arduino_Library.h"
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/SparkFun_u-blox_GNSS_Arduino_Library/src/u-blox_config_keys.h"
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/SparkFun_u-blox_GNSS_Arduino_Library/src/u-blox_structs.h"
)

target_include_directories(u_blox INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/SparkFun_u-blox_GNSS_Arduino_Library/src"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

target_link_libraries(u_blox INTERFACE
    arduino-compat
)