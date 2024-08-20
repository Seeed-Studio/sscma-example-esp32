

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

#define GRAVITY_EARTH               9.78762f
#define GEDAD_N_CHANNELS            3
#define GEDAD_PREDICT_DELAY_MS      (GYRO_SAMPLE_DELAY_MS / 2)
#define GEDAD_YIELD_DELAY_MS        4
#define GEDAD_RESCALE               true
#define GEDAD_RESCALE_SQUEEZE       (1024.0 * 0.005)
#define GEDAD_RESCALE_EXPAND        (200.0 / 1024.0)
#define GEDAD_CWT_ON_RAW            true
#define GEDAD_CWT_DOWNSAMPLE_FACTOR 0.125

#define GEDAD_NYQUIST_FREQ          200.0 / 2.0
#define GEDAD_CUTOFF_FREQ           90.0
#define GEDAD_NUM_TAPS              200
#define GEDAD_MTF_BINS              64
#define GEDAD_CWT_SCALES            16
#define GEDAD_CWT_SCALE_START       2.0
#define GEDAD_CWT_SCALE_SETP        1.0

#define GYRO_SAMPLE_DELAY_MS        5
#define GYRO_BUFFER_SIZE            6144
#define GYRO_VIEW_SIZE              256
#define GYRO_SAMPLE_SIZE_MIN        256
#define GYRO_SAMPLE_ALLOW_DUPLICATE true
#define GYRO_SAMPLE_MODE            0
#define GYRO_SAMPLE_DEBUG           0

#define GEDAD_STD \
    { 2.768257565258182, 2.768257565258182, 2.768257565258182 }
#define GEDAD_MEAN \
    { 5.096703647473537, 5.096703647473537, 5.096703647473537 }

#define TFLITE_TENSOR_ARENA_SIZE (1024 * 1024)

#define DEBUG                    3

static constexpr std::array<float, GEDAD_N_CHANNELS> gedadStd  = GEDAD_STD;
static constexpr std::array<float, GEDAD_N_CHANNELS> gedadMean = GEDAD_MEAN;

enum class GEDADState { Sampling, Predicting };

static volatile size_t                       gyroSampleCount = 0;
static volatile bool                         gyroSampleFlag  = false;
static ad::GEDADNN<float, GEDAD_N_CHANNELS>* gedad           = nullptr;

static void gyroSampleCallback(TimerHandle_t xTimer) {
    if (!gyroSampleFlag) [[unlikely]] {
        return;
    }

    static std::array<float, GEDAD_N_CHANNELS> frame;
#if GEDAD_N_CHANNELS == 3
    qma7981_get_acce(&frame[0], &frame[1], &frame[2]);
#else
    #error "Unsupported number of channels"
#endif

    auto        current_time     = std::chrono::high_resolution_clock::now();
    static auto last_sample_time = current_time;
    auto duration    = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_sample_time).count();
    last_sample_time = current_time;
#if GEDAD_N_CHANNELS == 3
    frame[0] *= GRAVITY_EARTH;
    frame[1] *= GRAVITY_EARTH;
    frame[2] *= GRAVITY_EARTH;
#else
    #error "Unsupported number of channels"
#endif

#if !GYRO_SAMPLE_DEBUG
    if (gedad != nullptr) [[likely]] {
        gedad->pushToBuffer(frame);
        gyroSampleCount = gyroSampleCount + 1;
    }
#endif

#if GYRO_SAMPLE_MODE == 1
    std::cout << std::fixed << std::setprecision(5);
    for (const auto& f : frame) {
        std::cout << f << " ";
    }
    std::cout << duration << std::endl;
#else
    #if DEBUG > 2
    if (gyroSampleCount < GYRO_SAMPLE_SIZE_MIN) {
        std::cout << std::fixed << std::setprecision(5) << "sample " << gyroSampleCount << ":" << std::endl;
        std::cout << "  ";
        for (const auto& f : frame) {
            std::cout << f << " ";
        }
        std::cout << duration << std::endl;    }
    #endif
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
    static auto    start             = std::chrono::high_resolution_clock::now();
    static auto    end               = std::chrono::high_resolution_clock::now();
    static size_t  last_sample_count = 0;
    static int32_t sample_count_diff = 0;

    while (true) {
#if !GYRO_SAMPLE_ALLOW_DUPLICATE
        while (gyroSampleCount % GYRO_VIEW_SIZE != 0) {
            vTaskDelay(pdMS_TO_TICKS(GEDAD_YIELD_DELAY_MS));
        }
#endif

        start                       = std::chrono::high_resolution_clock::now();
        sample_count_diff           = gyroSampleCount - last_sample_count;
        last_sample_count           = gyroSampleCount;
        const auto [l1, l2]         = gedad->predict(GYRO_VIEW_SIZE,
                                             GEDAD_RESCALE,
                                             GEDAD_RESCALE_SQUEEZE,
                                             GEDAD_RESCALE_EXPAND,
                                             GEDAD_STD,
                                             GEDAD_MEAN,
                                             GEDAD_CWT_ON_RAW,
                                             GEDAD_CWT_DOWNSAMPLE_FACTOR);
        end                         = std::chrono::high_resolution_clock::now();
        const bool has_data_skipped = sample_count_diff > GYRO_VIEW_SIZE;

        gedad->printPerf();
        gedad->printTFPerf();

        std::cout << "Predict results: " << l1 << " " << l2 << std::endl;
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Predict time: " << duration_ms << "ms" << std::endl;
        std::cout << "Sampled: " << sample_count_diff << " (" << gyroSampleCount << ")" << std::endl;
        std::cout << "Data skipped: " << (has_data_skipped ? "yes" : "no") << std::endl;
        std::cout << "Coverage: " << 1.0f - (float(sample_count_diff) / float(GYRO_VIEW_SIZE)) << std::endl;

        int32_t delay = GYRO_SAMPLE_DELAY_MS - duration_ms;
        if (delay > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay));
        } else {
            vTaskDelay(pdMS_TO_TICKS(GEDAD_YIELD_DELAY_MS));
        }
    }
}

#if GYRO_SAMPLE_MODE == 0
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
        gedad = new ad::GEDADNN<float, GEDAD_N_CHANNELS>(GYRO_BUFFER_SIZE,
                                                         TFLITE_TENSOR_ARENA_SIZE,
                                                         reinterpret_cast<void*>(nn_model_tflite),
                                                         GEDAD_NYQUIST_FREQ,
                                                         GEDAD_CUTOFF_FREQ,
                                                         GEDAD_NUM_TAPS,
                                                         GEDAD_MTF_BINS,
                                                         GEDAD_CWT_SCALES,
                                                         GEDAD_CWT_SCALE_START,
                                                         GEDAD_CWT_SCALE_SETP);
    }
    assert(gedad != nullptr);
    std::cout << "GEDAD initialized" << std::endl;
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // dry run model
    std::cout << "Dry run model..." << std::endl;
    std::cout << *gedad << std::endl;
    gedad->predict(GYRO_VIEW_SIZE,
                   GEDAD_RESCALE,
                   GEDAD_RESCALE_SQUEEZE,
                   GEDAD_RESCALE_EXPAND,
                   GEDAD_STD,
                   GEDAD_MEAN,
                   GEDAD_CWT_ON_RAW,
                   GEDAD_CWT_DOWNSAMPLE_FACTOR);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // start sampling
    std::cout << "Start sampling in 3 seconds";
    for (size_t i = 0; i < 3; ++i) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    gyroSampleFlag = true;
    #if GYRO_SAMPLE_DEBUG
    std::array<float, GEDAD_N_CHANNELS> frame;
    for (size_t i = 0; i < GYRO_VIEW_SIZE; ++i) {
        const auto v = std::sin((static_cast<float>(i) / static_cast<float>(GYRO_VIEW_SIZE)) * 2.0f *
                                static_cast<float>(dsp::constants::PI)) *
                       GRAVITY_EARTH;
        for (auto& f : frame) {
            f = v;
        }
        if (gedad != nullptr) [[likely]] {
            gedad->pushToBuffer(frame);
            gyroSampleCount = gyroSampleCount + 1;
        }
    }
    #endif
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
#else
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

    // start sampling
    std::cout << "Start sampling in 3 seconds";
    for (size_t i = 0; i < 3; ++i) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    gyroSampleFlag = true;

    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
#endif