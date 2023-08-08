# get IDF version for comparison
cmake_minimum_required(VERSION 3.5)
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

FILE(GLOB_RECURSE SRCS_EDGELAB
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

set(INCLUDES_EDGELAB
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ports
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ports/espressif
    ${CMAKE_CURRENT_SOURCE_DIR}/src/repl
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inference
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/JPEGENC
)

if(CONFIG_EL_LIB_FLASHDB)
    list(APPEND SRCS_EDGELAB
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
    list(APPEND INCLUDES_EDGELAB 
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB
    )
endif()

if(CONFIG_EL_LIB_JPEGENC)
    list(APPEND SRCS_EDGELAB 
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/JPEGENC/JPEGENC.cpp
    )
    list(APPEND INCLUDES_EDGELAB 
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/JPEGENC
    )
endif()

message(STATUS "SRCS_EDGELAB: ${SRCS_EDGELAB}")
message(STATUS "INCLUDES_EDGELAB: ${INCLUDES_EDGELAB}")

set(priv_req tflite-lib screen esp32-camera)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND priv_req esp_timer driver spi_flash esp_partition efuse)
endif()

idf_component_register(
            SRCS "${SRCS_EDGELAB}"
            INCLUDE_DIRS "${INCLUDES_EDGELAB}"
            REQUIRES ${pub_req}
            PRIV_REQUIRES ${priv_req})

