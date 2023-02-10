#include <math.h>
#include <stdint.h>
#include <forward_list>

#include "algo_fomo.hpp"
#include "fomo_model_data.h"

#include "fb_gfx.h"
#include "isp.h"
#include "base64.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "fomo";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gEvent = true;
static bool gReturnFB = true;
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
    camera_fb_t *frame = NULL;

    uint16_t h = input->dims->data[1];
    uint16_t w = input->dims->data[2];
    uint16_t c = input->dims->data[3];

    while (true)
    {
        if (gEvent)
        {
            if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
            {

                int dsp_start_time = esp_timer_get_time() / 1000;
                c = 1;
                if (c == 1)
                    rgb565_to_gray(input->data.uint8, frame->buf, frame->height, frame->width, h, w, ROTATION_UP);
                else if (c == 3)
                    rgb565_to_rgb888(input->data.uint8, frame->buf, frame->height, frame->width, h, w, ROTATION_UP);

                int dsp_end_time = esp_timer_get_time() / 1000;

                if (debug_mode)
                {
                    printf("Begin output\n");
                    printf("Format: {\"height\": %d, \"width\": %d, \"channels\": %d, \"model\": \"fomo\"}\r\n", h, w, c);
                    printf("Framebuffer: ");
                    base64_encode(input->data.uint8, input->bytes, putchar);
                    printf("\r\n");
                }

                for (int i = 0; i < input->bytes; i++)
                {
                    input->data.int8[i] = input->data.uint8[i] - 128;
                }

                // Run the model on this input and make sure it succeeds.
                int start_time = esp_timer_get_time() / 1000;

                if (kTfLiteOk != interpreter->Invoke())
                {
                    MicroPrintf("Invoke failed.");
                }

                int end_time = esp_timer_get_time() / 1000;

                vTaskDelay(10 / portTICK_PERIOD_MS);

                TfLiteTensor *output = interpreter->output(0);

                uint16_t n_h = output->dims->data[1]; // result in row
                uint16_t n_w = output->dims->data[2]; // result in col
                uint16_t n_t = output->dims->data[3]; // result in target

                printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", (dsp_end_time - dsp_start_time), (end_time - start_time), 0);
                bool found = false;
                for (int i = 0; i < n_h; i++)
                {

                    for (int j = 0; j < n_w; j++)
                    {
                        uint8_t max_conf = 0;
                        uint8_t max_target = 0;
                        for (int t = 0; t < n_t; t++)
                        {
                            uint8_t conf = (output->data.int8[i * n_w * n_t + j * n_t + t] - output->params.zero_point) * output->params.scale * 100;
                            if (conf > max_conf)
                            {
                                max_conf = conf;
                                max_target = t;
                            }
                        }
                        if (max_conf > 80 && max_target != 0)
                        {
                            found = true;
                            fomo_t obj;
                            obj.x = j * w / n_w + (w / n_w) / 2;
                            obj.y = i * h / n_h + (h / n_h) / 2;
                            obj.confidence = max_conf;
                            obj.target = max_target;

                            fb_gfx_drawCicle(frame, obj.x * (frame->width / w), obj.y * (frame->height / h), frame->width / n_w, 0xF800);
                            fb_gfx_printf(frame, obj.x, obj.y, 0x000F, "%s:%d", g_fomo_model_classes[obj.target - 1], obj.confidence);

                            printf("    %s (", g_fomo_model_classes[obj.target - 1]);
                            printf("%f", obj.confidence / 100.0f);
                            printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", obj.x, obj.y, w / n_w, h / n_h);
                        }
                    }
                }
                if (!found)
                {
                    printf("    No objects found\n");
                }
                if (debug_mode)
                {
                    printf("End output\n");
                }
            }

            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }

            if (xQueueResult)
            {
                xQueueSend(xQueueResult, NULL, portMAX_DELAY);
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

int register_algo_fomo(const QueueHandle_t frame_i,
                       const QueueHandle_t event,
                       const QueueHandle_t result,
                       const QueueHandle_t frame_o,
                       const bool camera_fb_return)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;
    gReturnFB = camera_fb_return;

    // get model (.tflite) from flash
    model = tflite::GetModel(g_fomo_model_data);
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

    static tflite::MicroMutableOpResolver<16> micro_op_resolver;
    micro_op_resolver.AddPad();
    micro_op_resolver.AddAdd();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddMean();
    micro_op_resolver.AddPack();
    micro_op_resolver.AddShape();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddSoftmax();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddStridedSlice();
    micro_op_resolver.AddConcatenation();
    micro_op_resolver.AddAveragePool2D();
    micro_op_resolver.AddDepthwiseConv2D();
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

    return 0;
}
