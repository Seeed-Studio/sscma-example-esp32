#pragma once

#include <cstdint>
#include <forward_list>
#include <sstream>
#include <string>

#include "el_algorithm.hpp"
#include "el_base64.h"
#include "el_cv.h"
#include "el_types.h"

using DELIM_F                = std::function<void(std::ostringstream& os)>;
static DELIM_F delim_f       = [](std::ostringstream& os) {};
static DELIM_F print_delim_f = [](std::ostringstream& os) { os << ", "; };
static DELIM_F print_void_f  = [](std::ostringstream& os) { delim_f = print_delim_f; };

#define DELIM_RESET \
    { delim_f = print_void_f; }
#define DELIM_PRINT(OS) \
    { delim_f(OS); }

inline uint32_t color_literal(uint8_t i) {
    static uint16_t color[] = {
      0x0000,
      0x03E0,
      0x001F,
      0x7FE0,
      0xFFFF,
    };
    return color[i % 5];
}

inline void draw_results_on_image(const std::forward_list<el_class_t>& results, el_img_t* img) {}

inline void draw_results_on_image(const std::forward_list<el_point_t>& results, el_img_t* img) {
    uint8_t i = 0;
    for (const auto& point : results) edgelab::el_draw_point(img, point.x, point.y, color_literal(++i));
}

inline void draw_results_on_image(const std::forward_list<el_box_t>& results, el_img_t* img) {
    uint8_t i = 0;
    for (const auto& box : results) {
        int16_t y = box.y - box.h / 2;
        int16_t x = box.x - box.w / 2;
        edgelab::el_draw_rect(img, x, y, box.w, box.h, color_literal(++i), 4);
    }
}

// TODO: avoid repeatly allocate/release memory in for loop
std::string el_img_2_base64_string(const el_img_t* img) {
    using namespace edgelab;
    if (!img) [[unlikely]]
        return {};
    size_t size    = img->width * img->height * 3;
    auto   rgb_img = el_img_t{.data   = new uint8_t[size]{},
                              .size   = size,
                              .width  = img->width,
                              .height = img->height,
                              .format = EL_PIXEL_FORMAT_RGB888,
                              .rotate = img->rotate};
    rgb_to_rgb(img, &rgb_img);

    auto          jpeg_img = el_img_t{.data   = new uint8_t[size]{},
                                      .size   = size,
                                      .width  = rgb_img.width,
                                      .height = rgb_img.height,
                                      .format = EL_PIXEL_FORMAT_JPEG,
                                      .rotate = rgb_img.rotate};
    el_err_code_t ret      = rgb_to_jpeg(&rgb_img, &jpeg_img);
    if (ret != EL_OK) [[unlikely]]
        return {};
    delete[] rgb_img.data;
    auto* buffer = new char[((jpeg_img.size + 2) / 3) * 4 + 1]{};
    el_base64_encode(jpeg_img.data, jpeg_img.size, buffer);
    delete[] jpeg_img.data;
    std::string data(buffer);
    delete[] buffer;
    return data;
}

template <typename AlgorithmType>
std::string invoke_results_2_string(AlgorithmType* algorithm,
                                    el_img_t*      img,
                                    uint8_t        algorithm_id,
                                    uint8_t        model_id,
                                    uint8_t        sensor_id,
                                    el_err_code_t  ret) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);
    os << "{\"algorithm_id\": " << unsigned(algorithm_id) << ", \"model_id\": " << unsigned(model_id)
       << ", \"sensor_id\": " << unsigned(sensor_id) << ", \"status\": " << int(ret)
       << ", \"preprocess_time\": " << unsigned(algorithm->get_preprocess_time())
       << ", \"run_time\": " << unsigned(algorithm->get_run_time())
       << ", \"postprocess_time\": " << unsigned(algorithm->get_postprocess_time())
       << ", \"results\": " << el_results_2_json(algorithm->get_results()) << ", \"data\": \""
       << el_img_2_base64_string(img) << "\"}\n";
    return std::string(os.str());
}
