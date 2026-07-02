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
set(CMAKE_PROJECT_NAME TI_FOC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



#收集所有lib下的文件
file(GLOB_RECURSE Lib_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/Core/Lib/**/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Core/Lib/**/*.cpp"
)

# 根据选项排除文件

list(FILTER Lib_SOURCES EXCLUDE REGEX ".*FOC_1/.*")
message(STATUS "FOC_1 module DISABLED")

#list(FILTER Lib_SOURCES EXCLUDE REGEX ".*FOC_2/.*")
#message(STATUS "FOC_2 module DISABLED")


# ---- 选择 app_task 版本（task_v1 = C, task_v2 = C++） ----
# 取消注释需要的版本，注释掉不需要的
list(FILTER Lib_SOURCES EXCLUDE REGEX ".*task_v1/.*")
message(STATUS "task_v1 (C version) DISABLED — using task_v2 (C++ version)")

#list(FILTER Lib_SOURCES EXCLUDE REGEX ".*task_v2/.*")
#message(STATUS "task_v2 (C++ version) DISABLED — using task_v1 (C version)")

# 禁用测试模块
list(FILTER Lib_SOURCES EXCLUDE REGEX ".*test_v3p/.*")
message(STATUS "test_v3p (motor test) DISABLED")

list(FILTER Lib_SOURCES EXCLUDE REGEX ".*trrry_cpp.*")
message(STATUS "trrry_cpp (empty test) DISABLED")


message(STATUS "Found Lib sources: ${Lib_SOURCES}")
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        ${Lib_SOURCES}
        FreeRTOSConfig.h
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
)

file(GLOB_RECURSE LIB_DIRS LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/Core/Lib/*")
set(LIB_DIRS "")
foreach(dir ${LIB_DIRS_LIST})
    if(IS_DIRECTORY ${dir})
        list(APPEND LIB_DIRS ${dir})
    endif()
endforeach()
# Add include paths
file(GLOB_RECURSE LIB_SUBDIRS LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/Core/Lib/*")

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${LIB_DIRS}
        ${LIB_SUBDIRS}
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
