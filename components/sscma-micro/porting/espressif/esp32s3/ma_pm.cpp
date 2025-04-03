#include <core/ma_compiler.h>
#include <core/ma_debug.h>

#include <porting/ma_osal.h>
#include <porting/ma_misc.h>

#include <cstring>

#include "ma_config_board.h"

#include "driver/temperature_sensor.h"
#include "esp_pm.h"

namespace ma {

temperature_sensor_handle_t temp_sensor = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);

static esp_pm_config_t pm_config = {
    .max_freq_mhz       = 240,
    .min_freq_mhz       = 80,
    .light_sleep_enable = true,
};

const static float thresh = 55.0;
static float tsens_out;

void ma_init_pm_ctrl() {
    MA_LOGD(MA_TAG, "Initializing Temperature sensor");

    int ret = temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    if (ret != ESP_OK) {
        MA_LOGE(MA_TAG, "Failed to install temperature sensor: %d", ret);
        return;
    }

    ret = temperature_sensor_enable(temp_sensor);
    if (ret != ESP_OK) {
        MA_LOGE(MA_TAG, "Failed to start temperature sensor: %d", ret);
        return;
    }

    MA_LOGD(MA_TAG, "Temperature sensor started");

    esp_pm_get_configuration(&pm_config);
    MA_LOGD(MA_TAG, "max_freq_mhz %d, min_freq_mhz %d, light_sleep_enable %d", pm_config.max_freq_mhz, pm_config.min_freq_mhz, pm_config.light_sleep_enable);
}

void ma_trigger_pm_ctrl() {
    if (!temp_sensor) {
        return;
    }
    temperature_sensor_get_celsius(temp_sensor, &tsens_out);

    if (tsens_out > thresh && pm_config.max_freq_mhz == 240) {
        pm_config.max_freq_mhz       = 80;
        pm_config.min_freq_mhz       = 80;
        pm_config.light_sleep_enable = true;
        int r                        = esp_pm_configure(&pm_config);
        MA_LOGD(MA_TAG, "Throuttle down to 80MHz, %d", r);
    } else if (tsens_out < thresh && pm_config.max_freq_mhz != 240) {
        pm_config.max_freq_mhz       = 240;
        pm_config.min_freq_mhz       = 80;
        pm_config.light_sleep_enable = true;
        int r                        = esp_pm_configure(&pm_config);
        MA_LOGD(MA_TAG, "Throuttle up to 240MHz, %d", r);
    }

    esp_pm_get_configuration(&pm_config);

    MA_LOGD(MA_TAG, "Temperature out celsius %f°C, max freq %dMHz", tsens_out, pm_config.max_freq_mhz);
}

}  // namespace ma
