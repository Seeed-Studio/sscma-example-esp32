#include <math.h>
#include <stdint.h>
#include <forward_list>

#include "algo_motion.hpp"
#include "motion_model_data.h"

#include "app_imu.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "motion";

static QueueHandle_t xQueueDataI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gEvent = true;
static bool debug_mode = false;

// Globals, used for compatibility with Arduino-style sketches.
namespace
{
    const tflite::Model *model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    TfLiteTensor *input = nullptr;

    // In order to use optimized tensorflow lite kernels, a signed int8_t quantized
    // model is preferred over the legacy unsigned model format. This means that
    // throughout this project, input images must be converted from unisgned to
    // signed format. The easiest and quickest way to convert from unsigned to
    // signed 8-bit integers is to subtract 128 from the unsigned value to get a
    // signed value.

#ifdef CONFIG_IDF_TARGET_ESP32S3
    constexpr int scratchBufSize = 1024 * 1024;
#else
    constexpr int scratchBufSize = 0;
#endif
    // An area of memory to use for input, output, and intermediate arrays.
    constexpr int kTensorArenaSize = 256 * 1024 + scratchBufSize;
    static uint8_t *tensor_arena; //[kTensorArenaSize]; // Maybe we should move this to external
} //

static void task_process_handler(void *arg)
{
    imu_data_t *data = NULL;

    while (true)
    {
        if (gEvent)
        {
            if (xQueueReceive(xQueueDataI, &data, portMAX_DELAY))
            {
                int dsp_start_time = esp_timer_get_time() / 1000;

                for (int i = 0; i < input->bytes; i++)
                {
                    input->data.int8[i] = (data->data[i]) / input->params.scale + input->params.zero_point;
                }

                int dsp_end_time = esp_timer_get_time() / 1000;

                // Run the model on this input and make sure it succeeds.
                int start_time = esp_timer_get_time() / 1000;

                if (kTfLiteOk != interpreter->Invoke())
                {
                    MicroPrintf("Invoke failed.");
                }

                int end_time = esp_timer_get_time() / 1000;

                TfLiteTensor *output = interpreter->output(0);

                printf("dsp time: %d ms, inference time: %d ms\n", dsp_end_time - dsp_start_time, end_time - start_time);
                printf("output: ");

                for (int i = 0; i < output->bytes; i++)
                {
                    printf("[%s : %f], ", g_motion_model_classes[i], (output->data.int8[i] - output->params.zero_point) * output->params.scale);
                }

                printf("\n");

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }
}

static void task_event_handler(void *arg)
{
    while (true)
    {
        xQueueReceive(xQueueEvent, &(gEvent), portMAX_DELAY);
    }
}

int register_algo_motion(const QueueHandle_t data_i,
                         const QueueHandle_t event,
                         const QueueHandle_t result)
{
    xQueueDataI = data_i;
    xQueueEvent = event;
    xQueueResult = result;

    // get model (.tflite) from flash
    model = tflite::GetModel(g_motion_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                    "version %d.",
                    model->version(), TFLITE_SCHEMA_VERSION);
        return -1;
    }

    if (tensor_arena == NULL)
    {
        tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (tensor_arena == NULL)
    {
        printf("Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
        return -1;
    }

    static tflite::MicroMutableOpResolver<3> micro_op_resolver;
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddSoftmax();
    micro_op_resolver.AddFullyConnected();

    // Build an interpreter to run the model with.
    // NOLINTNEXTLINE(runtime-global-variables)
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        MicroPrintf("AllocateTensors() failed");
        return -1;
    }

    // Get information about the memory area to use for the model's input.
    input = interpreter->input(0);

    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
    if (xQueueEvent)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);

    printf("algo_motion registered successfully\n");
    return 0;
}
