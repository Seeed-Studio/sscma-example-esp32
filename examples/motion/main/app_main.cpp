/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "app_imu.h"
#include "algo_motion.hpp"

static QueueHandle_t xQueueIMUData = NULL;

#define SAMPLE_FREQ_HZ 62.5
#define SAMPLE_WINDOW 1000

  


extern "C" void app_main()
{
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  xQueueIMUData = xQueueCreate(1, sizeof(imu_data_t *));
  register_imu(SAMPLE_WINDOW, SAMPLE_FREQ_HZ, xQueueIMUData);
  register_algo_motion(xQueueIMUData, NULL, NULL);
}
