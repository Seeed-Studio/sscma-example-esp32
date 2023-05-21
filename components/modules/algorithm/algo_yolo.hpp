#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

enum
{
    OBJECT_X_INDEX = 0,
    OBJECT_Y_INDEX = 1,
    OBJECT_W_INDEX = 2,
    OBJECT_H_INDEX = 3,
    OBJECT_C_INDEX = 4,
    OBJECT_T_INDEX = 5
};

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t w;
    uint8_t h;
    uint8_t confidence;
    uint8_t target;
} yolo_t;

int register_algo_yolo(const QueueHandle_t frame_i,
                       const QueueHandle_t event,
                       const QueueHandle_t result,
                       const QueueHandle_t frame_o,
                       const bool camera_fb_return);
