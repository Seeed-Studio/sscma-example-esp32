cmake_minimum_required(VERSION 3.5)
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

FILE(GLOB_RECURSE SSCMA_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/core/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sscma/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/porting/espressif/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/porting/espressif/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/porting/espressif/*.c
)

set(SSCMA_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/porting/espressif
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# FlashDB
list(APPEND SSCMA_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fal_partition.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fal_rtt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fdb_file.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fdb_kvdb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fdb_tsdb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fdb_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/fdb.c
)
list(APPEND SSCMA_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB
)

#JPEGENC
list(APPEND SSCMA_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/JPEGENC/JPEGENC.cpp
)
list(APPEND SSCMA_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/JPEGENC
)

message(STATUS "SSCMA_SRCS: ${SSCMA_SRCS}")
message(STATUS "SSCMA_INCLUDES: ${SSCMA_INCLUDES}")

set(priv_req esp-tflite-micro screen esp32-camera)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND priv_req esp_timer driver spi_flash esp_partition efuse esp_event esp_wifi mqtt nvs_flash)
endif()

idf_component_register(
    SRCS "${SSCMA_SRCS}"
    INCLUDE_DIRS "${SSCMA_INCLUDES}"
    REQUIRES ${pub_req}
    PRIV_REQUIRES ${priv_req}
)

target_compile_definitions(
    ${COMPONENT_TARGET}
    PUBLIC
    -DCONFIG_EL_TARGET_ESPPRESSIF
    -DCONFIG_EL_BOARD_SEEED_XIAO_ESP32S3
)
