#pragma once

#include <cstdint>
#include <forward_list>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

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
        return "\"ok\"";
    case EL_AGAIN:
        return "\"try again\"";
    case EL_ELOG:
        return "\"logic error\"";
    case EL_ETIMOUT:
        return "\"timeout\"";
    case EL_EIO:
        return "\"input/output error\"";
    case EL_EINVAL:
        return "\"invalid argument\"";
    case EL_ENOMEM:
        return "\"out of memory\"";
    case EL_EBUSY:
        return "\"busy\"";
    case EL_ENOTSUP:
        return "\"not supported\"";
    case EL_EPERM:
        return "\"operation not permitted\"";
    default:
        return "\"unknown\"";
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
        return "\"undefined\"";
    }
}

const char* algo_cat_2_str(el_algorithm_cat_t ac) {
    switch (ac) {
    case EL_ALGO_CAT_DET:
        return "\"detection\"";
    case EL_ALGO_CAT_POSE:
        return "\"pose\"";
    case EL_ALGO_CAT_CLS:
        return "\"classification\"";
    default:
        return "\"undefined\"";
    }
}

const char* sensor_type_2_str(el_sensor_type_t st) {
    switch (st) {
    case EL_SENSOR_TYPE_CAM:
        return "\"camera\"";
    default:
        return "\"undefined\"";
    }
}

const char* sensor_sta_2_str(el_sensor_state_t ss) {
    switch (ss) {
    case EL_SENSOR_STA_REG:
        return "\"registered\"";
    case EL_SENSOR_STA_AVAIL:
        return "\"available\"";
    case EL_SENSOR_STA_LOCKED:
        return "\"locked\"";
    default:
        return "\"unknown\"";
    }
}

const char* img_type_2_str(el_pixel_format_t pix_fmt) {
    switch (pix_fmt) {
    case EL_PIXEL_FORMAT_GRAYSCALE:
        return "\"grayscale\"";
    case EL_PIXEL_FORMAT_JPEG:
        return "\"jpeg\"";
    case EL_PIXEL_FORMAT_RGB565:
        return "\"rgb565\"";
    case EL_PIXEL_FORMAT_RGB888:
        return "\"rgb888\"";
    case EL_PIXEL_FORMAT_YUV422:
        return "\"yuv422\"";
    default:
        return "\"undefined\"";
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

template <typename T> constexpr std::string results_2_json(const std::forward_list<T>& results) {
    auto os{std::ostringstream(std::ios_base::ate)};
    using F                = std::function<void(void)>;
    static F delim_f       = []() {};
    static F print_delim_f = [&os]() { os << ", "; };
    static F print_void_f  = [&]() { delim_f = print_delim_f; };
    delim_f                = print_void_f;
    if constexpr (std::is_same<T, el_box_t>::value) {
        os << "\"boxes\": [";
        for (const auto& box : results) {
            delim_f();
            os << "{\"x\": " << static_cast<unsigned>(box.x) << ", \"y\": " << static_cast<unsigned>(box.y)
               << ", \"w\": " << static_cast<unsigned>(box.w) << ", \"h\": " << static_cast<unsigned>(box.h)
               << ", \"target\": " << static_cast<unsigned>(box.target)
               << ", \"score\": " << static_cast<unsigned>(box.score) << "}";
        }
    } else if constexpr (std::is_same<T, el_point_t>::value) {
        os << "\"points\": [";
        for (const auto& point : results) {
            delim_f();
            os << "{\"x\": " << static_cast<unsigned>(point.x) << ", \"y\": " << static_cast<unsigned>(point.y)
               << ", \"target\": " << static_cast<unsigned>(point.target) << "}";
        }
    } else if constexpr (std::is_same<T, el_class_t>::value) {
        os << "\"classes\": [";
        for (const auto& cls : results) {
            delim_f();
            os << "{\"score\": " << static_cast<unsigned>(cls.score)
               << ", \"target\": " << static_cast<unsigned>(cls.target) << "}";
        }
    }
    os << "]";
    return os.str();
}

// TODO: avoid repeatly allocate/release memory in for loop
std::string img_2_json_str(const el_img_t* img) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);

    if (!img || !img->data) [[unlikely]]
        return {};

    os << "\"jpeg\": \"";
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

template <typename ConfigType> std::string algorithm_config_2_json_str(const ConfigType& config) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);

    if constexpr (std::is_same<ConfigType, el_algorithm_fomo_config_t>::value ||
                  std::is_same<ConfigType, el_algorithm_imcls_config_t>::value)
        os << "\"score_threshold\": " << static_cast<unsigned>(config.score_threshold);
    else if constexpr (std::is_same<ConfigType, el_algorithm_yolo_config_t>::value)
        os << "\"score_threshold\": " << static_cast<unsigned>(config.score_threshold)
           << ", \"iou_threshold\": " << static_cast<unsigned>(config.iou_threshold);

    return os.str();
}

template <typename AlgorithmType>
std::string img_invoke_results_2_json_str(
  const AlgorithmType* algorithm, const el_img_t* img, const std::string& cmd, bool result_only, el_err_code_t ret) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);

    os << "\r{\"event\": {\"from\": \"" << cmd << "\", \"status\": " << err_code_2_str(ret)
       << ", \"contents\": {\"preprocess_time\": " << static_cast<unsigned>(algorithm->get_preprocess_time())
       << ", \"run_time\": " << static_cast<unsigned>(algorithm->get_run_time())
       << ", \"postprocess_time\": " << static_cast<unsigned>(algorithm->get_postprocess_time()) << ", "
       << results_2_json(algorithm->get_results()) << ", \"roi\": [" << static_cast<unsigned>(img->width) << ", "
       << static_cast<unsigned>(img->height) << "]";
    if (!result_only) os << ", " << img_2_json_str(img);
    os << "}}, \"timestamp\": " << el_get_time_ms() << "}\n";

    return std::string(os.str());
}
