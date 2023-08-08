#include <inttypes.h>
#include <stdio.h>

#include "edgelab.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "yolo_model_data.h"

#define kTensorArenaSize (1024 * 1024)

uint16_t color[] = {
  0x0000,
  0x03E0,
  0x001F,
  0x7FE0,
  0xFFFF,
};

extern "C" void app_main(void) {
    Device*  device  = Device::get_device();
    Display* display = device->get_display();
    Camera*  camera  = device->get_camera();

    camera->init(240, 240);
    display->init();
    auto* engine       = new InferenceEngine<EngineName::TFLite>();
    auto* tensor_arena = heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    engine->init(tensor_arena, kTensorArenaSize);

    auto* algorithm = new YOLO(engine);
    algorithm->init();

    while (1) {
        el_img_t img;
        camera->start_stream();
        camera->get_frame(&img);
        algorithm->run(&img);
        uint32_t preprocess_time  = algorithm->get_preprocess_time();
        uint32_t run_time         = algorithm->get_run_time();
        uint32_t postprocess_time = algorithm->get_postprocess_time();
        uint8_t  i                = 0;
        for (const auto box : algorithm->get_results()) {
            el_printf("\tbox -> cx_cy_w_h: [%d, %d, %d, %d] t: [%d] s: [%d]\n",
                      box.x,
                      box.y,
                      box.w,
                      box.h,
                      box.target,
                      box.score);

            uint16_t y = box.y - box.h / 2;
            uint16_t x = box.x - box.w / 2;
            el_draw_rect(&img, x, y, box.w, box.h, color[++i % 5], 4);
        }
        el_printf("preprocess: %d, run: %d, postprocess: %d\n", preprocess_time, run_time, postprocess_time);
        display->show(&img);
        camera->stop_stream();
    }

    for (int i = 1000; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
