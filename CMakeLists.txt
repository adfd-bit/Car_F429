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
set(CMAKE_PROJECT_NAME Car_F429)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    )

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/motor.c
    Core/Src/OPS9.c
    Core/Src/path_plan.c
    Core/Src/servo_motor.c
    Core/Src/ar_screen.c
    Core/Src/lub_cat.c
    Core/Inc/arm_control.h
    Core/Src/arm_control.c
    Core/Inc/cy_z.h
    Core/Src/cy_z.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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

# 让 printf/sprintf 支持 %f 浮点格式。
# 原因：cmake/gcc-arm-none-eabi.cmake 里用了 --specs=nano.specs（newlib-nano），
#       它为了省 Flash 默认把 printf 家族的浮点支持阉掉了，不加这行 "%f" 会静默输出空。
# 位置：必须写在本文件。cmake/*.cmake 每次 CubeMX 重新生成工程都会被覆盖，本文件不会。
# 代价：约 +2~3 KB Flash。若哪天要用 scanf 读 %f，再补一行 -u _scanf_float。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
