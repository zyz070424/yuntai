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
set(CMAKE_PROJECT_NAME yuntai)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Force response files for Ninja to avoid oversized command lines on Windows.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE ON CACHE BOOL "" FORCE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    user_file/1_middlewares/Driver/SPI/drv_spi.c
    user_file/1_middlewares/Driver/CAN/drv_can.c
    user_file/1_middlewares/Driver/USB/drv_usb.c
    user_file/1_middlewares/Driver/USART/drv_usart.c
    user_file/1_middlewares/Algorthm/PID/alg_pid.c
    user_file/1_middlewares/Algorthm/Matrix/alg_matrix.c
    user_file/1_middlewares/Algorthm/DWT/alg_dwt.c
    user_file/1_middlewares/Algorthm/Quaternion/alg_quaternion.c
    user_file/2_Device/BMI088/dvc_bmi088.c
    user_file/2_Device/Motor/dvc_motor.c
    user_file/2_Device/Motor/dvc_motor_protect.c
    user_file/2_Device/VOFA/dvc_vofa.c
    user_file/2_Device/DR16/dvc_dr16.c
    user_file/2_Device/Manifold/dvc_manifold.c
    user_file/3_Module/Gimbal/gimbal_sentry.c
    user_file/3_Module/Gimbal/gimbal_sentry_target.c
    user_file/3_Module/Gimbal/Gimbal.c
    user_file/4_Task/MyTask.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

    user_file/1_middlewares/Driver/SPI
    user_file/1_middlewares/Driver/CAN
    user_file/1_middlewares/Driver/USB
    user_file/1_middlewares/Driver/USART
    user_file/1_middlewares/Algorthm/PID
    user_file/1_middlewares/Algorthm/Matrix
    user_file/1_middlewares/Algorthm/DWT
    user_file/1_middlewares/Algorthm/Quaternion
    user_file/2_Device/BMI088
    user_file/2_Device/Motor
    user_file/2_Device/VOFA
    user_file/2_Device/DR16
    user_file/2_Device/Manifold
    user_file/3_Module/Gimbal
    user_file/4_Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)


