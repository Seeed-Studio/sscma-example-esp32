# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeLists.txt
     DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../components/sscma-micro)

set(EXTRA_COMPONENT_DIRS ../../components)
add_compile_options(-fdiagnostics-color=always )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(serial_echo_demo)
