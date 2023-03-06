#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

int register_algo_motion(const QueueHandle_t data_i,
                       const QueueHandle_t event,
                       const QueueHandle_t result);
