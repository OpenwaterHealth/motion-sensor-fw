cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME motion-sensor-fw)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    USB/Class/COMMS/Src/usbd_comms.c
    USB/Class/CompositeBuilder/Src/usbd_composite_builder.c
    USB/Class/HISTO/Src/usbd_histo.c
    USB/Class/IMU/Src/usbd_imu.c
    Core/Src/0X02C1B.c
    Core/Src/camera_manager.c
    Core/Src/crosslink.c
    Core/Src/histo_fake.c
    Core/Src/i2c_protocol.c
    Core/Src/i2c_master.c
    Core/Src/ICM20948.c
    Core/Src/if_commands.c
    Core/Src/lwrb.c
    Core/Src/jsmn.c
    Core/Src/logging.c
    Core/Src/uart_comms.c
    Core/Src/utils.c
    USB/Core/Src/usbd_ioreq.c
    USB/Core/Src/usbd_desc.c
    USB/Core/Src/usbd_ctlreq.c
    USB/Core/Src/usbd_core.c
    USB/Core/Src/usbd_conf.c
    USB/Core/Src/usb_device.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    USB/Core/Inc
    USB/Class/CompositeBuilder/Inc
    USB/Class/COMMS/Inc
    USB/Class/HISTO/Inc
    USB/Class/IMU/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USE_USBD_COMPOSITE
    USE_USB_HS
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate hex and bin files after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating .hex and .bin files"
)

