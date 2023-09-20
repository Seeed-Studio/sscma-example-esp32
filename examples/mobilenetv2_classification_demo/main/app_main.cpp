#include <inttypes.h>
#include <stdio.h>

#include "core/edgelab.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mobilenetv2_model_data.h"

#define kTensorArenaSize (1024 * 1024)

extern "C" void app_main(void) {
    using namespace edgelab;

    Device*  device  = Device::get_device();
    Display* display = device->get_display();
    Camera*  camera  = device->get_camera();

    display->init();
    camera->init(240, 240);

    auto* engine       = new EngineTFLite();
    auto* tensor_arena = heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    engine->init(tensor_arena, kTensorArenaSize);
    engine->load_model(g_mobilenetv2_model_data, g_mobilenetv2_model_data_len);
    auto* algorithm = new AlgorithmIMCLS(engine, 20); // we already have softmax

    while (true) {
        el_img_t img;
        camera->start_stream();
        camera->get_frame(&img);
        algorithm->run(&img);
        uint32_t preprocess_time  = algorithm->get_preprocess_time();
        uint32_t run_time         = algorithm->get_run_time();
        uint32_t postprocess_time = algorithm->get_postprocess_time();
        for (const auto& cls : algorithm->get_results()) {
            el_printf("\tclass -> t: [%s] s: [%d]\n", g_mobilenetv2_model_classes[cls.target], cls.score);
        }
        el_printf("preprocess: %d, run: %d, postprocess: %d\n", preprocess_time, run_time, postprocess_time);
        display->show(&img);
        camera->stop_stream();

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    delete algorithm;
    delete engine;
}
