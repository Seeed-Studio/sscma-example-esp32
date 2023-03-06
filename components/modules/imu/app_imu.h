#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "qma7981.h"

typedef struct imu_data
{
    float *data;
    int len;
    float sample_rate;
    uint32_t window;
} imu_data_t;

#ifdef __cplusplus
extern "C"
{
#endif


esp_err_t register_imu(
    const uint32_t window,
    const float sample_rate,
    const QueueHandle_t frame_o);

#ifdef __cplusplus
}
#endif

