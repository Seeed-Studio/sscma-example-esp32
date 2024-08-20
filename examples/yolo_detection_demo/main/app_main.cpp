#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "BYTETracker.h"
#include "core/algorithm/el_algorithm_yolo.h"
#include "core/engine/el_engine_tflite.h"
#include "core/utils/el_cv.h"
#include "freertos/FreeRTOS.h"
#include "porting/el_device.h"
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
    using namespace edgelab;

    Device*  device  = Device::get_device();
    Display* display = device->get_display();
    Camera*  camera  = device->get_camera();

    display->init();
    camera->init(0);

    auto* engine       = new EngineTFLite();
    auto* tensor_arena = heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    engine->init(tensor_arena, kTensorArenaSize);
    engine->load_model(g_yolo_model_data, g_yolo_model_data_len);
    auto* algorithm = new AlgorithmYOLO(engine);

    int fps = 30;

    uint32_t free_mem_init  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t rtos_heap_init = xPortGetFreeHeapSize();

    el_printf("Init Free memory: heap_caps %dK, ucHeap %dK\n",
              int(free_mem_init / 1024),
              int(rtos_heap_init / 1024));

    void* ptr = malloc(1024 * 10);
    el_printf("malloc 10k: %dK/%dK\n",
              int(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024),
              int(xPortGetFreeHeapSize() / 1024));
    free(ptr);

    ptr = new uint8_t[1024 * 10];
    el_printf("new 10k: %dK/%dK\n",
              int(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024),
              int(xPortGetFreeHeapSize() / 1024));
    delete[] static_cast<uint8_t*>(ptr);

    BYTETracker tracker(2, 60);

    while (true) {
        auto ss = std::chrono::high_resolution_clock::now();

        el_img_t img;
        camera->start_stream();
        camera->get_frame(&img);
        algorithm->run(&img);
        uint32_t preprocess_time  = algorithm->get_preprocess_time();
        uint32_t run_time         = algorithm->get_run_time();
        uint32_t postprocess_time = algorithm->get_postprocess_time();

        std::vector<BYTETracker::Object> objects;

        for (const auto& box : algorithm->get_results()) {
            BYTETracker::Object obj;

            obj.rect.x = static_cast<float>(box.x);
            obj.rect.y = static_cast<float>(box.y);

            obj.rect.width  = static_cast<float>(box.w);
            obj.rect.height = static_cast<float>(box.h);

            obj.label = box.target;

            obj.prob = static_cast<float>(box.score);

            objects.push_back(obj);
        }

        auto s = std::chrono::high_resolution_clock::now();

        std::vector<STrack> output_stracks = tracker.update(objects);

        auto e = std::chrono::high_resolution_clock::now();

        size_t time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count();

        el_printf("ByteTrack Time: %d\n", time_ms);

        for (int i = 0; i < output_stracks.size(); i++) {
            std::vector<float> tlwh = output_stracks[i].tlwh;

            bool vertical = tlwh[2] / tlwh[3] > 1.6;
            if (tlwh[2] * tlwh[3] > 20 && !vertical) {
                el_printf("\tbox -> x_y_w_h: [%d, %d, %d, %d] id: [%d] s: [%d] cat: [%d]\n",
                          static_cast<int>(round(tlwh[0])),
                          static_cast<int>(round(tlwh[1])),
                          static_cast<int>(round(tlwh[2])),
                          static_cast<int>(round(tlwh[3])),
                          output_stracks[i].track_id,
                          int(output_stracks[i].score),
                          output_stracks[i].label
                          );
            }
        }

        el_printf("preprocess: %d, run: %d, postprocess: %d\n", preprocess_time, run_time, postprocess_time);
        display->show(&img);
        camera->stop_stream();

        uint32_t free_mem  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        uint32_t rtos_heap = xPortGetFreeHeapSize();

        el_printf("Free memory: heap_caps %dK/%dK, ucHeap %dK/%dK\n",
                  int(free_mem / 1024),
                  int(free_mem_init / 1024),
                  int(rtos_heap / 1024),
                  int(rtos_heap_init / 1024));

        auto ee = std::chrono::high_resolution_clock::now();

        size_t tt = std::chrono::duration_cast<std::chrono::milliseconds>(ee - ss).count();

        el_printf("Total Time: %d\n", tt);
    }

    delete algorithm;
    delete engine;
}
