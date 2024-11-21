#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"

#include "sscma_client_io.h"
#include "sscma_client_ops.h"
#include "sscma_client_flasher.h"

#define TAG "sscma_client.ota"

static sscma_client_io_handle_t io = NULL;
sscma_client_handle_t client = NULL;

static sscma_client_io_handle_t flasher_io = NULL;
static sscma_client_flasher_handle_t flasher = NULL;

#define EXAMPLE_SAVE_IMAGE_TO_SD 0

void on_event(sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx)
{
    printf("on_event: %s\n", reply->data);
    return;
}

void on_log(sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx)
{
    if (reply->len >= 100)
    {
        strcpy(&reply->data[100 - 4], "...");
    }
    // Note: reply is automatically recycled after exiting the function.
    printf("log: %s\n", reply->data);
}

void on_connect(sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx)
{
    printf("on_connect\n");
}

esp_err_t bsp_spiffs_init(char *mount_point, size_t max_files)
{
    static bool inited = false;
    if (inited)
    {
        return ESP_OK;
    }
    esp_vfs_spiffs_conf_t conf = {
        .base_path = mount_point,
        .partition_label = "storage",
        .max_files = max_files,
        .format_if_mount_failed = false,
    };
    esp_vfs_spiffs_register(&conf);

    size_t total = 0, used = 0;
    esp_err_t ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ret_val;
}

void app_main(void)
{
    bsp_spiffs_init("/spiffs", 10);

    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(1, 8 * 1024, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(1, 21, 20, -1, -1));

    sscma_client_io_uart_config_t io_uart_config = {
        .user_ctx = NULL,
    };

    sscma_client_config_t sscma_client_config = SSCMA_CLIENT_CONFIG_DEFAULT();
    sscma_client_config.reset_gpio_num = GPIO_NUM_5;

    sscma_client_new_io_uart_bus((sscma_client_uart_bus_handle_t)1, &io_uart_config, &io);

    sscma_client_new(io, &sscma_client_config, &client);

    const sscma_client_callback_t callback = {
        .on_connect = on_connect,
        .on_event = on_event,
        .on_log = on_log,
    };

    if (sscma_client_register_callback(client, &callback, NULL) != ESP_OK)
    {
        printf("set callback failed\n");
        abort();
    }

    // flasher init

    const sscma_client_flasher_we2_config_t flasher_config = {
        .reset_gpio_num = GPIO_NUM_5,
        .io_expander = NULL,
        .flags.reset_use_expander = false,
        .flags.reset_high_active = false,
        .user_ctx = NULL,
    };

    sscma_client_new_flasher_we2_uart(io, &flasher_config, &flasher);

    sscma_client_init(client);
    sscma_client_set_model(client, 1);
     sscma_client_info_t *info;
     if (sscma_client_get_info(client, &info, true) == ESP_OK)
     {
         printf("ID: %s\n", (info->id != NULL) ? info->id : "NULL");
         printf("Name: %s\n", (info->name != NULL) ? info->name : "NULL");
         printf("Hardware Version: %s\n", (info->hw_ver != NULL) ? info->hw_ver : "NULL");
         printf("Software Version: %s\n", (info->sw_ver != NULL) ? info->sw_ver : "NULL");
         printf("Firmware Version: %s\n", (info->fw_ver != NULL) ? info->fw_ver : "NULL");
     }
     else
     {
         printf("get info failed\n");
     }
     sscma_client_model_t *model;
     if (sscma_client_get_model(client, &model, true) == ESP_OK)
     {
         printf("ID: %d\n", model->id ? model->id : -1);
         printf("UUID: %s\n", model->uuid ? model->uuid : "N/A");
         printf("Name: %s\n", model->name ? model->name : "N/A");
         printf("Version: %s\n", model->ver ? model->ver : "N/A");
         printf("URL: %s\n", model->url ? model->url : "N/A");
         printf("Checksum: %s\n", model->checksum ? model->checksum : "N/A");
         printf("Classes:\n");
         if (model->classes[0] != NULL)
         {
             for (int i = 0; model->classes[i] != NULL; i++)
             {
                 printf("  - %s\n", model->classes[i]);
             }
         }
         else
         {
             printf("  N/A\n");
         }
     }
     else
     {
         printf("get model failed\n");
     }

    int64_t start = esp_timer_get_time();
    if (sscma_client_ota_start(client, flasher, 0x000000) != ESP_OK)
    {
        ESP_LOGI(TAG, "sscma_client_ota_start failed\n");
    }
    else
    {
        ESP_LOGI(TAG, "sscma_client_ota_start success\n");
        FILE *f = fopen("/spiffs/firmware.img", "r");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "open file failed\n");
        }
        else
        {
            ESP_LOGI(TAG, "open file success\n");
            size_t len = 0;
            char buf[256] = { 0 };
            do
            {
                memset(buf, 0, sizeof(buf));
                if (fread(buf, 1, sizeof(buf), f) <= 0)
                {
                    printf("\n");
                    break;
                }
                else
                {
                    len += sizeof(buf);
                    if (sscma_client_ota_write(client, buf, sizeof(buf)) != ESP_OK)
                    {
                        ESP_LOGI(TAG, "sscma_client_ota_write failed\n");
                        break;
                    }
                }
            }
            while (true);
            fclose(f);
        }
        sscma_client_ota_finish(client);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "sscma_client_ota_finish success, take %lld us\n", esp_timer_get_time() - start);
    }

    if (sscma_client_invoke(client, -1, false, false) != ESP_OK)
    {
        printf("sample failed\n");
    }

    while (1)
    {
        printf("free_heap_size = %ld\n", esp_get_free_heap_size());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
