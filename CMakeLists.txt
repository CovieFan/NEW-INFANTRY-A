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
set(CMAKE_PROJECT_NAME NEW-INFANTRY-A)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 产物统一命名为 firmware.elf：让 .vscode 里的调试/烧录路径与工程名、文件夹名完全无关
# （改这一行 = 以后复制/改名工程都不用再动 launch.json / tasks.json）
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME firmware)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# ★ 自动收集 USER 下所有 .c（CONFIGURE_DEPENDS：增删/移动文件会自动重新配置）
file(GLOB_RECURSE USER_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/USER/*.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SRC})

# Add include paths
# ★ 每个含 .c 的目录都作为头文件搜索路径（改目录结构后不再需要手工维护）
set(USER_INC_DIRS "")
foreach(_src ${USER_SRC})
    get_filename_component(_dir ${_src} DIRECTORY)
    list(APPEND USER_INC_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES USER_INC_DIRS)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${USER_INC_DIRS})

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
