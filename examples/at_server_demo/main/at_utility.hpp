#pragma once

#include <cstdint>
#include <forward_list>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include "at_definations.hpp"
#include "el_algorithm.hpp"
#include "el_base64.h"
#include "el_cv.h"
#include "el_data.hpp"
#include "el_device_esp.h"
#include "el_repl.hpp"
#include "el_types.h"

std::string string_2_str(const std::string& str) {
    std::string ss(1, '"');
    for (char c : str) {
        if (c == '"') [[unlikely]]
            ss += "\\\"";
        else if (c == '\n') [[unlikely]]
            ss += "\\n";
        else if (std::isprint(c)) [[likely]]
            ss += c;
    }
    ss += '"';
    return ss;
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
        int16_t y = box.y - (box.h >> 1);
        int16_t x = box.x - (box.w >> 1);
        edgelab::el_draw_rect(img, x, y, box.w, box.h, color_literal(++i), 4);
    }
}

std::string model_info_2_json(el_model_info_t model_info) {
    auto os{std::ostringstream(std::ios_base::ate)};

    os << "{\"id\": " << static_cast<unsigned>(model_info.id)
       << ", \"type\": " << static_cast<unsigned>(model_info.type) << ", \"address\": \"0x" << std::hex
       << static_cast<unsigned>(model_info.addr_flash) << "\", \"size\": \"0x" << static_cast<unsigned>(model_info.size)
       << "\"}";

    return std::string(os.str());
}

std::string sensor_info_2_json(el_sensor_info_t sensor_info) {
    auto os{std::ostringstream(std::ios_base::ate)};

    os << "{\"id\": " << static_cast<unsigned>(sensor_info.id)
       << ", \"type\": " << static_cast<unsigned>(sensor_info.type)
       << ", \"state\": " << static_cast<unsigned>(sensor_info.state) << "}";

    return std::string(os.str());
}

template <typename T> constexpr std::string results_2_json(const std::forward_list<T>& results) {
    auto os{std::ostringstream(std::ios_base::ate)};

    DELIM_RESET;
    if constexpr (std::is_same<T, el_box_t>::value) {
        os << "\"boxes\": [";
        for (const auto& box : results) {
            DELIM_PRINT(os);
            os << "[" << static_cast<unsigned>(box.x) << ", " << static_cast<unsigned>(box.y) << ", "
               << static_cast<unsigned>(box.w) << ", " << static_cast<unsigned>(box.h) << ", "
               << static_cast<unsigned>(box.target) << ", " << static_cast<unsigned>(box.score) << "]";
        }
    } else if constexpr (std::is_same<T, el_point_t>::value) {
        os << "\"points\": [";
        for (const auto& point : results) {
            DELIM_PRINT(os);
            os << "[" << static_cast<unsigned>(point.x) << ", " << static_cast<unsigned>(point.y) << ", "
               << static_cast<unsigned>(point.target) << "]";
        }
    } else if constexpr (std::is_same<T, el_class_t>::value) {
        os << "\"classes\": [";
        for (const auto& cls : results) {
            DELIM_PRINT(os);
            os << "[" << static_cast<unsigned>(cls.score) << ", " << static_cast<unsigned>(cls.target) << "]";
        }
    }
    os << "]";

    return std::string(os.str());
}

// TODO: avoid repeatly allocate/release memory in for loop
std::string img_2_jpeg_json_str(const el_img_t* img) {
    using namespace edgelab;
    auto os = std::ostringstream(std::ios_base::ate);

    if (!img || !img->data) [[unlikely]]
        return {};

    os << "\"image\": \"";
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

template <typename InfoConfType> std::string algorithm_info_and_conf_2_json_str(const InfoConfType& info_and_conf) {
    using namespace edgelab;
    auto os{std::ostringstream(std::ios_base::ate)};

    os << "{\"type\": " << static_cast<unsigned>(info_and_conf.info.type)
       << ", \"categroy\": " << static_cast<unsigned>(info_and_conf.info.categroy)
       << ", \"input_from\": " << static_cast<unsigned>(info_and_conf.info.input_from) << ", \"config\": {";
    if constexpr (std::is_same<InfoConfType, el_algorithm_fomo_config_t>::value ||
                  std::is_same<InfoConfType, el_algorithm_imcls_config_t>::value)
        os << "\"tscore\": " << static_cast<unsigned>(info_and_conf.score_threshold);
    else if constexpr (std::is_same<InfoConfType, el_algorithm_yolo_config_t>::value)
        os << "\"tscore\": " << static_cast<unsigned>(info_and_conf.score_threshold)
           << ", \"tiou\": " << static_cast<unsigned>(info_and_conf.iou_threshold);
    os << "}}";

    return std::string(os.str());
}

template <typename AlgorithmType>
std::string img_invoke_results_2_json_str(
  const AlgorithmType* algorithm, const el_img_t* img, const std::string& cmd, bool result_only, el_err_code_t ret) {
    using namespace edgelab;
    auto os{std::ostringstream(std::ios_base::ate)};

    os << REPLY_EVT_HEADER << "\"name\": \"" << cmd << "\", \"code\": " << static_cast<int>(ret)
       << ", \"data\": {\"perf\": [" << static_cast<unsigned>(algorithm->get_preprocess_time()) << ", "
       << static_cast<unsigned>(algorithm->get_run_time()) << ", "
       << static_cast<unsigned>(algorithm->get_postprocess_time()) << "], " << results_2_json(algorithm->get_results());
    if (!result_only) os << ", " << img_2_jpeg_json_str(img);
    os << "}}\n";

    return std::string(os.str());
}

template <typename AlgorithmType> class AlgorithmConfigHelper {
   public:
    using ConfigType = typename AlgorithmType::ConfigType;

    AlgorithmConfigHelper(AlgorithmType* algorithm)
        : _instance(edgelab::ReplDelegate::get_delegate()->get_server_handler()),
          _algorithm(algorithm),
          _config(algorithm->get_algorithm_config()),
          _kv(el_make_storage_kv_from_type(_config)),
          _storage(edgelab::DataDelegate::get_delegate()->get_storage_handler()),
          _serial(edgelab::Device::get_device()->get_serial()) {
        using namespace edgelab;

        if (_storage->contains(_kv.key)) [[likely]]
            *_storage >> _kv;
        else
            *_storage << _kv;
        _algorithm->set_algorithm_config(_kv.value);

        if constexpr (std::is_same<ConfigType, el_algorithm_fomo_config_t>::value ||
                      std::is_same<ConfigType, el_algorithm_imcls_config_t>::value ||
                      std::is_same<ConfigType, el_algorithm_yolo_config_t>::value) {
            el_err_code_t ret = _instance->register_cmd(
              "TSCORE", "Set score threshold", "SCORE_THRESHOLD", [this](std::vector<std::string> argv) {
                  auto          os    = std::ostringstream(std::ios_base::ate);
                  uint8_t       value = std::atoi(argv[1].c_str());
                  el_err_code_t ret   = value <= 100 ? EL_OK : EL_EINVAL;

                  if (ret == EL_OK) {
                      this->_algorithm->set_score_threshold(value);
                      this->_kv.value.score_threshold = value;
                      *(this->_storage) << this->_kv;
                  }

                  os << REPLY_CMD_HEADER << "\"name\": \"" << argv[0] << "\", \"code\": " << static_cast<int>(ret)
                     << ", \"data\": " << static_cast<unsigned>(this->_kv.value.score_threshold) << "}\n";

                  const auto& str{os.str()};
                  this->_serial->send_bytes(str.c_str(), str.size());

                  return EL_OK;
              });
            if (ret == EL_OK) _config_cmds.emplace_front("TSCORE");

            ret = _instance->register_cmd("TSCORE?", "Get score threshold", "", [this](std::vector<std::string> argv) {
                auto os = std::ostringstream(std::ios_base::ate);

                os << REPLY_CMD_HEADER << "\"name\": \"" << argv[0] << "\", \"code\": " << static_cast<int>(EL_OK)
                   << ", \"data\": " << static_cast<unsigned>(this->_algorithm->get_score_threshold()) << "}\n";

                const auto& str{os.str()};
                this->_serial->send_bytes(str.c_str(), str.size());

                return EL_OK;
            });
            if (ret == EL_OK) _config_cmds.emplace_front("TSCORE?");
        }
        if constexpr (std::is_same<ConfigType, el_algorithm_yolo_config_t>::value) {
            el_err_code_t ret = _instance->register_cmd(
              "TIOU", "Set IoU threshold", "IOU_THRESHOLD", [this](std::vector<std::string> argv) {
                  auto          os    = std::ostringstream(std::ios_base::ate);
                  uint8_t       value = std::atoi(argv[1].c_str());
                  el_err_code_t ret   = value <= 100 ? EL_OK : EL_EINVAL;

                  if (ret == EL_OK) {
                      this->_algorithm->set_iou_threshold(value);
                      this->_kv.value.iou_threshold = value;
                      *(this->_storage) << this->_kv;
                  }

                  os << REPLY_CMD_HEADER << "\"name\": \"" << argv[0] << "\", \"code\": " << static_cast<int>(ret)
                     << ", \"data\": " << static_cast<unsigned>(this->_kv.value.iou_threshold) << "}\n";

                  const auto& str{os.str()};
                  this->_serial->send_bytes(str.c_str(), str.size());

                  return EL_OK;
              });
            if (ret == EL_OK) _config_cmds.emplace_front("TIOU");

            ret = _instance->register_cmd("TIOU?", "Get IoU threshold", "", [this](std::vector<std::string> argv) {
                auto os = std::ostringstream(std::ios_base::ate);

                os << REPLY_CMD_HEADER << "\"name\": \"" << argv[0] << "\", \"code\": " << static_cast<int>(EL_OK)
                   << ", \"data\": " << static_cast<unsigned>(this->_algorithm->get_iou_threshold()) << "}\n";

                const auto& str{os.str()};
                this->_serial->send_bytes(str.c_str(), str.size());

                return EL_OK;
            });
            if (ret == EL_OK) _config_cmds.emplace_front("TIOU?");
        }
    }

    ConfigType dump_config() { return _config; }

    ~AlgorithmConfigHelper() {
        for (const auto& cmd : _config_cmds) _instance->unregister_cmd(cmd);
    }

   private:
    edgelab::repl::ReplServer*     _instance;
    std::forward_list<std::string> _config_cmds;

    AlgorithmType*                                     _algorithm;
    ConfigType                                         _config;
    edgelab::data::types::el_storage_kv_t<ConfigType&> _kv;

    edgelab::data::Storage* _storage;
    edgelab::Serial*        _serial;
};

std::vector<std::string> tokenize_function_2_argv(const std::string& input) {
    std::vector<std::string> argv;

    size_t index = 0;
    size_t size  = input.size();
    char   c     = {};

    while (index < size) {
        c = input.at(index);
        if (std::isalnum(c) || c == '_') {
            size_t prev = index;
            while (++index < size) {
                c = input.at(index);
                if (!std::isalnum(c) && c != '_') [[unlikely]]
                    break;
            }
            argv.push_back(input.substr(prev, index - prev));
        } else
            ++index;
    }
    argv.shrink_to_fit();

    return argv;
}
