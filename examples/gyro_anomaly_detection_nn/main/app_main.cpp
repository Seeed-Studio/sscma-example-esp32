

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <qma7981.h>

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>

#include "GEDAD.hpp"
#include "nn_model.h"

#define GRAVITY_EARTH            9.78762f
#define GYRO_SAMPLE_DELAY_MS     5
#define GEDAD_PREDICT_DELAY_MS   (GYRO_SAMPLE_DELAY_MS / 2)
#define GEDAD_YIELD_DELAY_MS     5

#define GYRO_BUFFER_SIZE         6144
#define GYRO_VIEW_SIZE           2048
#define GYRO_SAMPLE_SIZE_MIN     2048

#define TFLITE_TENSOR_ARENA_SIZE (512 * 1024)

#define DEBUG                    3

enum class GEDADState { Sampling, Predicting };

static volatile size_t        gyroSampleCount = 0;
static volatile bool          gyroSampleFlag  = false;
static ad::GEDADNN<float, 3>* gedad           = nullptr;

static void gyroSampleCallback(TimerHandle_t xTimer) {
    if (!gyroSampleFlag) [[unlikely]] {
        return;
    }

    static std::array<float, 3> xyz;
    qma7981_get_acce(&xyz[0], &xyz[1], &xyz[2]);
    xyz[0] *= GRAVITY_EARTH;
    xyz[1] *= GRAVITY_EARTH;
    xyz[2] *= GRAVITY_EARTH;

    if (gedad != nullptr) [[likely]] {
        gedad->pushToBuffer(xyz);
        gyroSampleCount = gyroSampleCount + 1;
    }
#if DEBUG > 2
    if (gyroSampleCount < GYRO_SAMPLE_SIZE_MIN) {
        std::cout << std::fixed << std::setprecision(5) << "sample " << gyroSampleCount << ":\n  x: " << xyz[0]
                  << " y: " << xyz[1] << " z: " << xyz[2] << std::endl;
    }
#endif
}

static void gyroSensorInit() {
    qma7981_init();
    qma7981_set_range(QMA_RANGE_8G);

    static TimerHandle_t gyroSampleTimer =
      xTimerCreate("GyroSampleTimer", pdMS_TO_TICKS(GYRO_SAMPLE_DELAY_MS), pdTRUE, nullptr, gyroSampleCallback);
    if (gyroSampleTimer == nullptr) {
        std::cout << "Failed to create gyro sample timer" << std::endl;
    }
    if (xTimerStart(gyroSampleTimer, 0) != pdPASS) {
        std::cout << "Failed to start gyro sample timer" << std::endl;
    }

    std::cout << "Gyro sensor initialized" << std::endl;
}

static void gedadPredictTask(void*) {
    static auto start             = std::chrono::high_resolution_clock::now();
    static auto end               = std::chrono::high_resolution_clock::now();
    static auto last_sample_count = 0;

    while (true) {
        start                        = std::chrono::high_resolution_clock::now();
        const auto sample_count_diff = gyroSampleCount - last_sample_count;
        last_sample_count            = gyroSampleCount;
        const auto [l1, l2]          = gedad->predict(GYRO_VIEW_SIZE);
        end                          = std::chrono::high_resolution_clock::now();
        const bool has_data_skipped  = sample_count_diff > GYRO_VIEW_SIZE;

        gedad->printPerf();

        std::cout << "Predict loss: " << l1 << " " << l2 << std::endl;
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Predict time: " << duration_ms << "ms" << std::endl;
        std::cout << "Sampled " << gyroSampleCount << " samples" << std::endl;
        std::cout << "Data skipped: " << (has_data_skipped ? "yes" : "no") << std::endl;

        int32_t delay = GYRO_SAMPLE_DELAY_MS - duration_ms;
        if (delay > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay));
        } else {
            vTaskDelay(pdMS_TO_TICKS(GEDAD_YIELD_DELAY_MS));
        }
    }
}

extern "C" void app_main() {
    // print heap info
    multi_heap_info_t heapInfo;
    heap_caps_get_info(&heapInfo, MALLOC_CAP_8BIT);
    std::cout << "Heap info:" << std::endl;
    std::cout << "  total: " << heapInfo.total_allocated_bytes << " bytes" << std::endl;
    std::cout << "  free: " << heapInfo.total_free_bytes << " bytes" << std::endl;
    std::cout << "  minimum free: " << heapInfo.minimum_free_bytes << " bytes" << std::endl;
    std::cout << "  largest free block: " << heapInfo.largest_free_block << " bytes" << std::endl;
    std::cout << std::endl;

    // initialize gyro sensor
    std::cout << "Initializing gyro sensor..." << std::endl;
    gyroSensorInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // initialize GEDAD
    std::cout << "Initializing GEDAD..." << std::endl;
    if (gedad == nullptr) [[likely]] {
        gedad = new ad::GEDADNN<float, 3>(
          GYRO_BUFFER_SIZE, TFLITE_TENSOR_ARENA_SIZE, reinterpret_cast<void*>(nn_model_tflite));
    }
    assert(gedad != nullptr);
    std::cout << "GEDAD initialized" << std::endl;
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // dry run model
    std::cout << "Dry run model..." << std::endl;
    std::cout << *gedad << std::endl;
    gedad->predict(GYRO_VIEW_SIZE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // start sampling
    std::cout << "Start sampling in 3 seconds";
    for (size_t i = 0; i < 3; ++i) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    gyroSampleFlag = true;
    while (gyroSampleCount < GYRO_SAMPLE_SIZE_MIN) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    std::cout << "Sampled " << gyroSampleCount << " samples" << std::endl;
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // start prediction task
    std::cout << "Start prediction loop..." << std::endl;
    xTaskCreate(gedadPredictTask, "GEDADPredictTask", 8192, nullptr, 1, nullptr);

    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
