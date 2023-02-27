add_library(lora-e32 INTERFACE)

target_sources(lora-e32 INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/LoRa_E32_Series_Library/LoRa_E32.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/LoRa_E32_Series_Library/LoRa_E32.h"
)

target_include_directories(lora-e32 INTERFACE
 "${CMAKE_CURRENT_LIST_DIR}/../../Arduino/LoRa_E32_Series_Library"
)

if (NOT TARGET arduino-compat)
    add_subdirectory(../../pico-arduino-compat/arduino-compat build_arduino-compat)
endif()

# add_subdirectory(../../pico-arduino-compat/libs/printoo_library build_printoo)

target_link_libraries(lora-e32 INTERFACE
    arduino-compat
    # pac-printoo_library
)



