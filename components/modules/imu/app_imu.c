#include "app_imu.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "app_imu";

#define GRAVITY_EARTH 9.80665f

static QueueHandle_t xQueueDataO = NULL;

static void task_process_handler(void *arg)
{
    imu_data_t *imu_data = (imu_data_t *)arg;
    uint32_t interval = 1000 / imu_data->sample_rate;

    while (true)
    {
        for (int i = 0; i < imu_data->len; i += 3)
        {
            float x, y, z;
            qma7981_get_acce(&x, &y, &z);
            imu_data->data[i] = x * GRAVITY_EARTH;
            imu_data->data[i + 1] = y * GRAVITY_EARTH;
            imu_data->data[i + 2] = z * GRAVITY_EARTH;
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }
        xQueueSend(xQueueDataO, &imu_data, portMAX_DELAY);
    }
}

esp_err_t register_imu(
    const uint32_t window,
    const float sample_rate,
    const QueueHandle_t frame_o)
{
    ESP_LOGI(TAG, "IMU module is %s", "qma7981");

    qma7981_init();
    qma7981_set_range(QMA_RANGE_8G);
    imu_data_t *imu_data = (imu_data_t *)malloc(sizeof(imu_data_t));
    imu_data->sample_rate = sample_rate;
    imu_data->window = window;
    imu_data->len = (window * (uint32_t)sample_rate) / 1000 * 3;
    imu_data->data = (float *)malloc(imu_data->len * sizeof(float));

    xQueueDataO = frame_o;

    xTaskCreatePinnedToCore(task_process_handler, "task_process", 1024, imu_data, 5, NULL, 0);
    return ESP_OK;
}