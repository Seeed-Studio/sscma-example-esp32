cmake_minimum_required(VERSION 3.12.4)

message(STATUS "Building for ESP environment")

if(NOT DEFINED SSCMA_PORTING_ROOT_DIR)
    set(SSCMA_PORTING_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
    set(SSCMA_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../sscma-micro)
endif()


file(GLOB_RECURSE SRCS_CORE
    ${SSCMA_ROOT_DIR}/sscma/core/*.c
    ${SSCMA_ROOT_DIR}/sscma/core/*.cpp
)

file(GLOB_RECURSE SRCS_SERVER
    ${SSCMA_ROOT_DIR}/sscma/server/*.c
    ${SSCMA_ROOT_DIR}/sscma/server/*.cpp
    ${SSCMA_ROOT_DIR}/sscma/server/*.hpp
)

file(GLOB_RECURSE SRCS_PORTING
    ${SSCMA_ROOT_DIR}/sscma/porting/*.c
    ${SSCMA_ROOT_DIR}/sscma/porting/*.cpp
    ${SSCMA_ROOT_DIR}/sscma/porting/osal/*.c
    ${SSCMA_ROOT_DIR}/sscma/porting/osal/*.cpp

    ${SSCMA_PORTING_ROOT_DIR}/porting/*.c
    ${SSCMA_PORTING_ROOT_DIR}/porting/*.cpp
)

file(GLOB_RECURSE SRCS_THIRDPARTY
    ${SSCMA_ROOT_DIR}/3rdparty/littlefs/littlefs/lfs.c
    ${SSCMA_ROOT_DIR}/3rdparty/littlefs/littlefs/lfs_util.c

    ${SSCMA_ROOT_DIR}/3rdparty/JPEGENC/*.c
    ${SSCMA_ROOT_DIR}/3rdparty/JPEGENC/*.cpp
)

set(srcs
    ${SRCS_CORE}
    ${SRCS_SERVER}
    ${SRCS_PORTING}
    ${SRCS_THIRDPARTY}
)

set(includes
    ${SSCMA_ROOT_DIR}/sscma
    ${SSCMA_ROOT_DIR}/sscma/core
    ${SSCMA_ROOT_DIR}/sscma/porting
    ${SSCMA_ROOT_DIR}/sscma/porting/drivers
    ${SSCMA_ROOT_DIR}/sscma/server
    ${SSCMA_ROOT_DIR}/sscma/server/at
    ${SSCMA_ROOT_DIR}/sscma/server/at/codec
    ${SSCMA_ROOT_DIR}/sscma/server/at/callback

    ${SSCMA_ROOT_DIR}/3rdparty/JPEGENC
    ${SSCMA_ROOT_DIR}/3rdparty/littlefs/littlefs

    ${SSCMA_PORTING_ROOT_DIR}/porting
    ${SSCMA_PORTING_ROOT_DIR}/porting/espressif
    ${SSCMA_PORTING_ROOT_DIR}/porting/espressif/esp32s3
)

set(require
    json
    mbedtls
    esp_timer
    esp-tflite-micro
    esp32-camera
    esp_timer
    freertos
    driver
    spi_flash 
    esp_partition 
    efuse
)

set(priv_requires 
    driver
    mqtt
    nvs_flash
    esp_event
    esp_netif
    esp_wifi
    lwip
    mdns
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${includes}
    REQUIRES ${require}
    PRIV_REQUIRES ${priv_requires}
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC MA_PORTING_SEEED_XIAO_ESP32S3=1)

