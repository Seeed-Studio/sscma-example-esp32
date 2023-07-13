#include <inttypes.h>
#include <stdio.h>

#include "edgelab.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "meter.h"
#include "yolo_model_data.h"

#define kTensorArenaSize (1024 * 1024)

uint16_t color[] = {
    0x0000,
    0x001F,
    0x03E0,
    0x7FE0,
    0xFFFF,
};

extern "C" void app_main(void)
{
    Device *device = Device::get_device();
    Display *display = device->get_display();
    Camera *camera = device->get_camera();

    camera->init(240, 240);
    display->init();

    InferenceEngine *engine = new TFLiteEngine();
    uint8_t *tensor_arena =
        (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    engine->init(tensor_arena, kTensorArenaSize);
    engine->load_model(g_yolo_model_data, g_yolo_model_data_len);
    Algorithm *algorithm = new Yolo(*engine);

    algorithm->init();

    EL_LOGI("done");
    while (1) {
        el_img_t img;
        camera->start_stream();
        camera->get_frame(&img);
        // algorithm->run(&img);
        // uint32_t preprocess_time = algorithm->get_preprocess_time();
        // uint32_t run_time = algorithm->get_run_time();
        // uint32_t postprocess_time = algorithm->get_postprocess_time();

        // for (int i = 0; i < algorithm->get_result_size(); i++) {
        //     const el_box_t *box = static_cast<const el_box_t *>(algorithm->get_result(i));
        //     EL_LOGI("box: %d, %d, %d, %d, %d, %d",
        //             box->x,
        //             box->y,
        //             box->w,
        //             box->h,
        //             box->target,
        //             box->score);
        //     uint16_t x = box->x - box->w / 2;
        //     uint16_t y = box->y - box->h / 2;
        //     el_draw_rect(&img, x, y, box->w, box->h, color[i % 5], 4);
        // }
        // // EL_LOGI("draw done");
        // EL_LOGI("preprocess: %d, run: %d, postprocess: %d",
        //         preprocess_time,
        //         run_time,
        //         postprocess_time);
        display->show(&img);
        camera->stop_stream();
        EL_LOGI(".");
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
