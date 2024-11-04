idf_component_register(
    SRCS app_main.cpp
    REQUIRES freertos esp_psram
    PRIV_REQUIRES esp-tflite-micro esp-nn imu esp_timer driver spi_flash esp_partition efuse
)

add_compile_options(-fdiagnostics-color=always -ffast-math -O3)
