cmake_minimum_required(VERSION 3.5)
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

file(GLOB_RECURSE SSCMA_SRCS
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
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# FlashDB
file(GLOB_RECURSE SSCMA_SRCS_FDB
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB/*.c
)
list(APPEND SSCMA_SRCS ${SSCMA_SRCS_FDB})
list(APPEND SSCMA_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/FlashDB
)

# lwip
# file(GLOB_RECURSE SSCMA_SRCS_LWIP
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/api/*.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/core/*.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/ports/sys/*.c
# )
# list(APPEND SSCMA_SRCS ${SSCMA_SRCS_LWIP})
# list(APPEND SSCMA_INCLUDES
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/include/lwip
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/ports
#     ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lwip/ports/sys
# )

message(STATUS "SSCMA_SRCS: ${SSCMA_SRCS}")
message(STATUS "SSCMA_INCLUDES: ${SSCMA_INCLUDES}")

set(priv_req esp-tflite-micro screen esp32-camera)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND priv_req esp_timer driver spi_flash esp_partition efuse nvs_flash esp_event esp_netif esp_wifi lwip)
endif()

idf_component_register(
            SRCS "${SSCMA_SRCS}"
            INCLUDE_DIRS "${SSCMA_INCLUDES}"
            REQUIRES ${pub_req}
            PRIV_REQUIRES ${priv_req})
