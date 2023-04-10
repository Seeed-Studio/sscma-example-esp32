#pragma once

#include <stdint.h>
#include "esp_log.h"
#include "screen_driver.h"

#if CONFIG_LCD_MODULE_ESP_S3_EYE
#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 43
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define BOARD_LCD_ROTATE 0
#define LCD_HOST SPI2_HOST
#elif CONFIG_LCD_MODULE_XIAO_S3
#define BOARD_LCD_MOSI 9
#define BOARD_LCD_MISO 8
#define BOARD_LCD_SCK 7
#define BOARD_LCD_CS 2
#define BOARD_LCD_DC 4
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL -1
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define BOARD_LCD_ROTATE SCR_DIR_TBRL
#define LCD_HOST SPI2_HOST
#else
#error "No LCD module selected"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t register_lcd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb);

    void app_lcd_set_color(int color);

#ifdef __cplusplus
}
#endif
