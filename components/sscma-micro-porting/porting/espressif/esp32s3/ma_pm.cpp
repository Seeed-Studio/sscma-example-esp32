#include <core/ma_compiler.h>
#include <core/ma_debug.h>

#include <porting/ma_osal.h>

#include <cstring>

#include "ma_config_board.h"

#include "driver/temp_sensor.h"
#include "esp_pm.h"

namespace ma {

static esp_pm_config_t pm_config = {
    .max_freq_mhz       = 240,
    .min_freq_mhz       = 80,
    .light_sleep_enable = true,
};

const static float thresh = 55.0;
static float tsens_out;

void ma_init_pm_ctrl() {
    MA_LOGD(MA_TAG, "Initializing Temperature sensor");

    
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);

    MA_LOGD(MA_TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);

    temp_sensor.dac_offset = TSENS_DAC_DEFAULT;  // DEFAULT: range:-10℃ ~  80℃, error < 1℃.

    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();

    MA_LOGD(MA_TAG, "Temperature sensor started");

    esp_pm_get_configuration(&pm_config);
    MA_LOGD(MA_TAG, "max_freq_mhz %d, min_freq_mhz %d, light_sleep_enable %d", pm_config.max_freq_mhz, pm_config.min_freq_mhz, pm_config.light_sleep_enable);
}

void ma_trigger_pm_ctrl() {
    temp_sensor_read_celsius(&tsens_out);

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

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

}  // namespace ma