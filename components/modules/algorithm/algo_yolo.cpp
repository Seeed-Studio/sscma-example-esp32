#include <math.h>
#include <stdint.h>
#include <forward_list>

#include "algo_yolo.hpp"
#include "yolo_model_data.h"

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

static const char *TAG = "yolo";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gEvent = true;
static bool gReturnFB = true;
static bool debug_mode = false;

#define CONFIDENCE 40
#define IOU 30

std::forward_list<yolo_t> nms_get_obeject_topn(int8_t *dataset, uint16_t top_n, uint8_t threshold, uint8_t nms, uint16_t width, uint16_t height, int num_record, int8_t num_class, float scale, int zero_point);

// Globals, used for compatibility with Arduino-style sketches.
namespace
{
    const tflite::Model *model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    TfLiteTensor *input = nullptr;
    static std::forward_list<yolo_t> _yolo_list;

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
                _yolo_list.clear();

                if (c == 1)
                    rgb565_to_gray(input->data.uint8, frame->buf, frame->height, frame->width, h, w, ROTATION_UP);
                else if (c == 3)
                    rgb565_to_rgb888(input->data.uint8, frame->buf, frame->height, frame->width, h, w, ROTATION_UP);

                // for (int i = 0; i < input->bytes; i++)
                // {
                //     frame->buf[i] = input->data.uint8[i];
                // }

                int dsp_end_time = esp_timer_get_time() / 1000;

                if (debug_mode)
                {
                    printf("Begin output\n");
                    printf("Format: {\"height\": %d, \"width\": %d, \"channels\": %d, \"model\": \"yolo\"}\r\n", h, w, c);
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

                // Get the results of the inference attempt
                float scale = ((TfLiteAffineQuantization *)(output->quantization.params))->scale->data[0];
                int zero_point = ((TfLiteAffineQuantization *)(output->quantization.params))->zero_point->data[0];

                uint32_t records = output->dims->data[1];
                uint32_t num_class = output->dims->data[2] - OBJECT_T_INDEX;
                int16_t num_element = num_class + OBJECT_T_INDEX;

                _yolo_list = nms_get_obeject_topn(output->data.int8, records, CONFIDENCE, IOU, frame->width, frame->height, records, num_class, scale, zero_point);

                printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", (dsp_end_time - dsp_start_time), (end_time - start_time), 0);
                bool found = false;

#if 0
    for (int i = 0; i < records; i++)
    {
        float confidence = float(output->data.int8[i * num_element + OBJECT_C_INDEX] - zero_point) * scale;
        if (confidence > .40)
        {
            int8_t max = -128;
            int target = 0;
            for (int j = 0; j < num_class; j++)
            {
                if (max < output->data.int8[i * num_element + OBJECT_T_INDEX + j])
                {
                    max = output->data.int8[i * num_element + OBJECT_T_INDEX + j];
                    target = j;
                }
            }
            int x = int(float(float(output->data.int8[i * num_element + OBJECT_X_INDEX] - zero_point) * scale) * frame->width);
            int y = int(float(float(output->data.int8[i * num_element + OBJECT_Y_INDEX] - zero_point) * scale) * frame->height);
            int w = int(float(float(output->data.int8[i * num_element + OBJECT_W_INDEX] - zero_point) * scale) * frame->width);
            int h = int(float(float(output->data.int8[i * num_element + OBJECT_H_INDEX] - zero_point) * scale) * frame->height);

            printf("index: %d target: %d max: %d confidence: %d box{x: %d, y: %d, w: %d, h: %d}\n", i, target, max, int((float)confidence * 100), x, y, w, h);
        }
    }
#endif

                if (std::distance(_yolo_list.begin(), _yolo_list.end()) > 0)
                {
                    found = true;
                    printf("    Objects found: %d\n", std::distance(_yolo_list.begin(), _yolo_list.end()));
                    printf("    Objects:\n");
                    printf("    [\n");
                    for (auto &yolo : _yolo_list)
                    {
                        fb_gfx_drawRect(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h/2, yolo.w, yolo.h, 0x1FE0);
                        printf("        {\"class\": \"%d\", \"x\": %d, \"y\": %d, \"w\": %d, \"h\": %d, \"confidence\": %d},\n", yolo.target, yolo.x, yolo.y, yolo.w, yolo.h, yolo.confidence);
                    }
                    printf("    ]\n");
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

int register_algo_yolo(const QueueHandle_t frame_i,
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
    model = tflite::GetModel(g_yolo_model_data);
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

    static tflite::MicroMutableOpResolver<14> micro_op_resolver;
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddPad();
    micro_op_resolver.AddAdd();
    micro_op_resolver.AddSub();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddConcatenation();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddTranspose();
    micro_op_resolver.AddLogistic();
    micro_op_resolver.AddMul();
    micro_op_resolver.AddStridedSlice();
    micro_op_resolver.AddResizeNearestNeighbor();

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

#define CLIP(x, y, z) (x < y) ? y : ((x > z) ? z : x)

static bool _object_comparator_reverse(yolo_t &oa, yolo_t &ob)
{
    return oa.confidence < ob.confidence;
}

static bool _object_nms_comparator(yolo_t &oa, yolo_t &ob)
{
    return oa.confidence > ob.confidence;
}

static bool _object_comparator(yolo_t &oa, yolo_t &ob)
{
    return oa.x < ob.x;
}

bool _object_remove(yolo_t &obj)
{
    return (obj.confidence == 0);
}

static uint16_t _overlap(float x1, float w1, float x2, float w2)
{
    uint16_t l1 = x1 - w1 / 2;
    uint16_t l2 = x2 - w2 / 2;
    uint16_t left = l1 > l2 ? l1 : l2;
    uint16_t r1 = x1 + w1 / 2;
    uint16_t r2 = x2 + w2 / 2;
    uint16_t right = r1 < r2 ? r1 : r2;
    return right - left;
}

void _soft_nms_obeject_detection(std::forward_list<yolo_t> &yolo_obj_list, uint8_t nms)
{
    std::forward_list<yolo_t>::iterator max_box_obj;
    yolo_obj_list.sort(_object_comparator);
    for (std::forward_list<yolo_t>::iterator it = yolo_obj_list.begin(); it != yolo_obj_list.end(); ++it)
    {
        uint16_t area = it->w * it->h;
        for (std::forward_list<yolo_t>::iterator itc = std::next(it, 1); itc != yolo_obj_list.end(); ++itc)
        {
            if (itc->confidence == 0)
            {
                continue;
            }
            uint16_t iw = _overlap(itc->x, itc->w, it->x, it->w);
            if (iw > 0)
            {
                uint16_t ih = _overlap(itc->y, itc->h, it->y, it->h);
                if (ih > 0)
                {
                    float ua = float(itc->w * itc->h + area - iw * ih);
                    float ov = iw * ih / ua;
                    if (int(float(ov) * 100) >= nms)
                    {
                        itc->confidence = 0;
                    }
                }
            }
        }
    }
    yolo_obj_list.remove_if(_object_remove);
    return;
}

void _hard_nms_obeject_count(std::forward_list<yolo_t> &yolo_obj_list, uint8_t nms)
{
    std::forward_list<yolo_t>::iterator max_box_obj;
    yolo_obj_list.sort(_object_nms_comparator);
    for (std::forward_list<yolo_t>::iterator it = yolo_obj_list.begin(); it != yolo_obj_list.end(); ++it)
    {
        uint16_t area = it->w * it->h;
        for (std::forward_list<yolo_t>::iterator itc = std::next(it, 1); itc != yolo_obj_list.end(); ++itc)
        {
            if (itc->confidence == 0)
            {
                continue;
            }
            uint16_t iw = _overlap(itc->x, itc->w, it->x, it->w);
            if (iw > 0)
            {
                uint16_t ih = _overlap(itc->y, itc->h, it->y, it->h);
                if (ih > 0)
                {
                    float ua = float(itc->w * itc->h + area - iw * ih);
                    float ov = iw * ih / ua;
                    if (int(float(ov) * 100) >= nms)
                    {
                        itc->confidence = 0;
                    }
                }
            }
        }
    }
    yolo_obj_list.remove_if(_object_remove);

    return;
}

std::forward_list<yolo_t> nms_get_obeject_topn(int8_t *dataset, uint16_t top_n, uint8_t threshold, uint8_t nms, uint16_t width, uint16_t height, int num_record, int8_t num_class, float scale, int zero_point)
{
    std::forward_list<yolo_t> yolo_obj_list[num_class];
    int16_t num_obj[num_class] = {0};
    int16_t num_element = num_class + OBJECT_T_INDEX;
    for (int i = 0; i < num_record; i++)
    {
        float confidence = float(dataset[i * num_element + OBJECT_C_INDEX] - zero_point) * scale;
        if (int(float(confidence) * 100) >= threshold)
        {
            yolo_t obj;
            int8_t max = -128;
            obj.target = 0;

            for (int j = 0; j < num_class; j++)
            {
                if (max < dataset[i * num_element + OBJECT_T_INDEX + j])
                {
                    max = dataset[i * num_element + OBJECT_T_INDEX + j];
                    obj.target = j;
                }
            }

            
            int x = int(float(float(dataset[i * num_element + OBJECT_X_INDEX] - zero_point) * scale) * width);
            int y = int(float(float(dataset[i * num_element + OBJECT_Y_INDEX] - zero_point) * scale) * height);
            int w = int(float(float(dataset[i * num_element + OBJECT_W_INDEX] - zero_point) * scale) * width);
            int h = int(float(float(dataset[i * num_element + OBJECT_H_INDEX] - zero_point) * scale) * height);

            obj.x = CLIP(x, 0, width);
            obj.y = CLIP(y, 0, height);
            obj.w = CLIP(w, 0, width);
            obj.h = CLIP(h, 0, height);
            obj.confidence = int(float(confidence) * 100);
            if (num_obj[obj.target] >= top_n)
            {
                yolo_obj_list[obj.target].sort(_object_comparator_reverse);
                if (obj.confidence > yolo_obj_list[obj.target].front().confidence)
                {
                    yolo_obj_list[obj.target].pop_front();
                    yolo_obj_list[obj.target].emplace_front(obj);
                }
            }
            else
            {
                yolo_obj_list[obj.target].emplace_front(obj);
                num_obj[obj.target]++;
            }
        }
    }

    std::forward_list<yolo_t> result;

    for (int i = 0; i < num_class; i++)
    {
        if (!yolo_obj_list[i].empty())
        {
            _soft_nms_obeject_detection(yolo_obj_list[i], nms);
            result.splice_after(result.before_begin(), yolo_obj_list[i]);
        }
    }

    result.sort(_object_comparator); // left to right

    return result;
}