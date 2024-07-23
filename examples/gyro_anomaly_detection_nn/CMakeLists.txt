# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ../../components)
set(COMPONENTS imu driver cxx main bootloader bootloader_support app_trace esp_adc bus cmock console xtensa partition_table log freertos hal)

# print COMPONENTS
message(STATUS "COMPONENTS: ${COMPONENTS}")

add_compile_options(-fdiagnostics-color=always -ffast-math -O3)

# use c++20
set(CMAKE_CXX_STANDARD 20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(GyroAnomalyDetectionNN)

