

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <qma7981.h>

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>

#include "GEDAD.hpp"

#define GRAVITY_EARTH             9.78762f
#define GYRO_SAMPLE_DELAY_MS      16
#define GYRO_BUFFER_SIZE          512
#define GYRO_FIT_SAMPLE_SIZE_MIN  512

#define GEDAD_WINDOW_START        0
#define GEDAD_WINDOW_SIZE         192
#define GEDAD_SAMPLE_START        64
#define GEDAD_SAMPLE_SIZE         (64 + 192)
#define GEDAD_VIEW_SIZE           64
#define GEDAD_BATCH_SIZE          20
#define GEDAD_SHIFT_DIST          1
#define GEDAD_MINIMAL_N           5
#define GEDAD_PREDICT_DELAY_MS    (GYRO_SAMPLE_DELAY_MS / 2)

#define GEDAD_ALPHA               1.60f
#define GEDAD_ANOMALITY_TOLERANCE 0.15f

#define DEBUG                     3

enum class GEDADState { Sampling, Finetuning, Predicting };

static volatile size_t            gyroSampleCount  = 0;
static volatile bool              gyroSampleFlag   = false;
static volatile bool              gedadPredictFlag = false;
static ad::GEDAD<float, float, 3> gedad(GYRO_BUFFER_SIZE);

static void gyroSampleCallback(TimerHandle_t xTimer) {
    if (!gyroSampleFlag) [[unlikely]] {
        gedadPredictFlag = true;
        return;
    }

    static std::array<float, 3> xyz;
    qma7981_get_acce(&xyz[0], &xyz[1], &xyz[2]);
    xyz[0] *= GRAVITY_EARTH;
    xyz[1] *= GRAVITY_EARTH;
    xyz[2] *= GRAVITY_EARTH;

    gedad.pushToBuffer(xyz);
    gyroSampleCount = gyroSampleCount + 1;

#if DEBUG > 2
    if (gyroSampleCount < GYRO_FIT_SAMPLE_SIZE_MIN) {
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
    static auto start        = std::chrono::high_resolution_clock::now();
    static auto end          = std::chrono::high_resolution_clock::now();
    static auto anomaly_type = ad::AnomalyType::Unknown;

    while (1) {
        start        = std::chrono::high_resolution_clock::now();
        anomaly_type = gedad.isLastViewAnomaly(GEDAD_ANOMALITY_TOLERANCE);
        end          = std::chrono::high_resolution_clock::now();

        switch (anomaly_type) {
        case ad::AnomalyType::Anomaly:
            std::cout << "Anomaly detected" << std::endl;
            break;
        case ad::AnomalyType::Normal:
            std::cout << "Normal" << std::endl;
            break;
        case ad::AnomalyType::Unknown:
            std::cout << "Unknown (skipped)" << std::endl;
            break;
        }
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Predict time: " << duration_ms << "ms" << std::endl;
        std::cout << std::endl;

        int32_t delay = GYRO_SAMPLE_DELAY_MS - duration_ms;
        if (delay > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay));
        } else {
            vTaskDelay(pdMS_TO_TICKS(GEDAD_PREDICT_DELAY_MS));
        }
    }
}

extern "C" void app_main() {
    // initialize gyro sensor
    std::cout << "Initializing gyro sensor..." << std::endl;
    gyroSensorInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // start sampling
    std::cout << "Start sampling in 3 seconds";
    for (size_t i = 0; i < 3; ++i) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    gyroSampleFlag = true;
    while (gyroSampleCount < GYRO_FIT_SAMPLE_SIZE_MIN) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    gyroSampleFlag = false;
    while (!gedadPredictFlag) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    std::cout << "Sampling finished" << std::endl;
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // finetune the collected data
    std::cout << "Finetuning the collected data..." << std::endl;
    static std::array<ad::AlphaBeta<float>, 3> calibration = {
      ad::AlphaBeta<float>{GEDAD_ALPHA, 0.f},
      ad::AlphaBeta<float>{GEDAD_ALPHA, 0.f},
      ad::AlphaBeta<float>{GEDAD_ALPHA, 0.f}
    };
    gedad.calEuclideanDistThresh(GEDAD_WINDOW_START,
                                 GEDAD_WINDOW_SIZE,
                                 GEDAD_SAMPLE_START,
                                 GEDAD_SAMPLE_SIZE,
                                 GEDAD_VIEW_SIZE,
                                 GEDAD_BATCH_SIZE,
                                 GEDAD_SHIFT_DIST,
                                 GEDAD_MINIMAL_N,
                                 calibration);
    std::cout << "Finetuning finished" << std::endl;
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // start prediction task
    std::cout << "Start prediction loop...";
    gyroSampleFlag = true;
    xTaskCreate(gedadPredictTask, "GEDADPredictTask", 8192, nullptr, 1, nullptr);

    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
