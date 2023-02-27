cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../pico-sdk")

# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_lora)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


add_compile_definitions(ARDUINO_ARCH_STM32)
add_compile_definitions(FREQUENCY_868)
add_compile_definitions(LoRa_E32_DEBUG)



add_subdirectory( "${CMAKE_CURRENT_LIST_DIR}/../../libs/Lora_E32"
    build_Lora_E32
)

# rest of your project


if (TARGET tinyusb_device)
    add_executable(pico_lora
            main.cpp
            )

    # pull in common dependencies
    target_link_libraries(pico_lora 
        pico_stdlib
        pico_multicore
    )
    

    # enable usb output, disable uart output
    pico_enable_stdio_usb(pico_lora 1)
    pico_enable_stdio_uart(pico_lora 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(pico_lora)

    # add url via pico_set_program_url
    # example_auto_set_url(hello_world)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_world because TinyUSB submodule is not initialized in the SDK")
endif()


target_link_libraries(pico_lora 
    lora-e32
)
