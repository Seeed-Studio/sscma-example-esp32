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

const char* err_code_2_str(el_err_code_t ec) {
    switch (ec) {
    case EL_OK:
        return "\"OK\"";
    case EL_AGAIN:
        return "\"Try again\"";
    case EL_ELOG:
        return "\"Logic error\"";
    case EL_ETIMOUT:
        return "\"Timeout\"";
    case EL_EIO:
        return "\"I/O error\"";
    case EL_EINVAL:
        return "\"Invalid argument\"";
    case EL_ENOMEM:
        return "\"Out of memory\"";
    case EL_EBUSY:
        return "\"Busy\"";
    case EL_ENOTSUP:
        return "\"Not supported\"";
    case EL_EPERM:
        return "\"Operation not permitted\"";
    default:
        return "\"Unknown\"";
    }
}

const char* algo_type_2_str(el_algorithm_type_t at) {
    switch (at) {
    case EL_ALGO_TYPE_FOMO:
        return "\"FOMO\"";
    case EL_ALGO_TYPE_PFLD:
        return "\"PFLD\"";
    case EL_ALGO_TYPE_YOLO:
        return "\"YOLO\"";
    case EL_ALGO_TYPE_IMCLS:
        return "\"IMCLS\"";
    default:
        return "\"Undefined\"";
    }
}

const char* algo_cat_2_str(el_algorithm_cat_t ac) {
    switch (ac) {
    case EL_ALGO_CAT_DET:
        return "\"Detection\"";
    case EL_ALGO_CAT_POSE:
        return "\"Pose\"";
    case EL_ALGO_CAT_CLS:
        return "\"Classification\"";
    default:
        return "\"Undefined\"";
    }
}

const char* sensor_type_2_str(el_sensor_type_t st) {
    switch (st) {
    case EL_SENSOR_TYPE_CAM:
        return "\"Camera\"";
    default:
        return "\"Undefined\"";
    }
}

const char* sensor_sta_2_str(el_sensor_state_t ss) {
    switch (ss) {
    case EL_SENSOR_STA_REG:
        return "\"Registered\"";
    case EL_SENSOR_STA_AVAIL:
        return "\"Available\"";
    case EL_SENSOR_STA_LOCKED:
        return "\"Locked\"";
    default:
        return "\"Unknown\"";
    }
}

const char* img_type_2_str(el_pixel_format_t pix_fmt) {
    switch (pix_fmt) {
    case EL_PIXEL_FORMAT_GRAYSCALE:
        return "\"GrayScale\"";
    case EL_PIXEL_FORMAT_JPEG:
        return "\"JPEG\"";
    case EL_PIXEL_FORMAT_RGB565:
        return "\"RGB565\"";
    case EL_PIXEL_FORMAT_RGB888:
        return "\"RGB888\"";
    case EL_PIXEL_FORMAT_YUV422:
        return "\"YUV422\"";
    default:
        return "\"Undefined\"";
    }
}

inline uint32_t color_literal(uint8_t i) {
    static const uint16_t color[] = {
      0x0000,
      0x03E0,
      0x001F,
      0x7FE0,
      0xFFFF,
    };
    return color[i % 5];
}

void draw_results_on_image(const std::forward_list<el_class_t>& results, el_img_t* img) {}

void draw_results_on_image(const std::forward_list<el_point_t>& results, el_img_t* img) {
    uint8_t i = 0;
    for (const auto& point : results) edgelab::el_draw_point(img, point.x, point.y, color_literal(++i));
}

void draw_results_on_image(const std::forward_list<el_box_t>& results, el_img_t* img) {
    uint8_t i = 0;
    for (const auto& box : results) {
        int16_t y = box.y - box.h / 2;
        int16_t x = box.x - box.w / 2;
        edgelab::el_draw_rect(img, x, y, box.w, box.h, color_literal(++i), 4);
    }
}

// TODO: avoid repeatly allocate/release memory in for loop
std::string img_2_json_str(const el_img_t* img, bool info_only = false) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);

    if (!img || !img->data) [[unlikely]]
        return {};

    os << "\"info\": [" << img->width << ", " << img->height << "], \"JPEG\": \"";
    if (!info_only) {
        size_t size     = img->width * img->height * 3;
        auto   jpeg_img = el_img_t{.data   = new uint8_t[size]{},
                                   .size   = size,
                                   .width  = img->width,
                                   .height = img->height,
                                   .format = EL_PIXEL_FORMAT_JPEG,
                                   .rotate = img->rotate};

        el_err_code_t ret = rgb_to_jpeg(img, &jpeg_img);
        if (ret == EL_OK) [[likely]] {
            auto* buffer = new char[((jpeg_img.size + 2) / 3) * 4 + 1]{};
            el_base64_encode(jpeg_img.data, jpeg_img.size, buffer);
            os << buffer;
            delete[] buffer;
        }

        delete[] jpeg_img.data;
    }
    os << "\"";

    return std::string(os.str());
}

std::string simple_reply_ok(const std::string& cmd) {
    std::string str{};
    str += "{\"";
    str += cmd;
    str += "\": {\"status\": \"OK\"}}";
    return str;
}

template <typename AlgorithmType>
std::string invoke_results_2_json_str(const AlgorithmType* algorithm,
                                      const std::string&   sample_data_str,
                                      const std::string&   cmd,
                                      uint8_t              model_id,
                                      uint8_t              sensor_id,
                                      el_err_code_t        ret) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);

    os << "\r{\"event\": {\"from\": \"" << cmd << "\", \"status\": " << err_code_2_str(ret)
       << ", \"contents\": {\"model_id\": " << unsigned(model_id) << ", \"sensor_id\": " << unsigned(sensor_id)
       << ", \"preprocess_time\": " << unsigned(algorithm->get_preprocess_time())
       << ", \"run_time\": " << unsigned(algorithm->get_run_time())
       << ", \"postprocess_time\": " << unsigned(algorithm->get_postprocess_time()) << ", "
       << el_results_2_json(algorithm->get_results()) << ", \"data\": {" << sample_data_str
       << "}}}, \"timestamp\": " << el_get_time_ms() << "}\n";

    return std::string(os.str());
}
