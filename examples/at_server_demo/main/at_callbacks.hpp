#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <forward_list>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

#include "at_utility.hpp"
#include "edgelab.h"
#include "el_device_esp.h"

#define kTensorArenaSize (1024 * 1024)

void at_get_device_id() {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);
    os << "{\"id\": \"" << std::uppercase << std::hex << device->get_device_id()
       << std::resetiosflags(std::ios_base::basefield) << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_device_name() {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);
    os << "{\"name\": \"" << device->get_device_name() << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_device_status(int32_t boot_count,
                          uint8_t current_algorithm_id,
                          uint8_t current_model_id,
                          uint8_t current_sensor_id) {
    auto* serial = Device::get_device()->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);
    os << "{\"stat\": " << int(EL_OK) << ", \"boot_count\": " << unsigned(boot_count)
       << ", \"current_algorithm_id\": " << unsigned(current_algorithm_id)
       << ", \"current_model_id\": " << unsigned(current_model_id)
       << ", \"current_sensor_id\": " << unsigned(current_sensor_id) << ", \"timestamp\": " << el_get_time_ms()
       << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_version() {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);
    os << "{\"version\": \"v" << EL_VERSION << "\", \"hardware\": \"v" << unsigned(device->get_chip_revision_id())
       << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_available_algorithms() {
    auto* serial                = Device::get_device()->get_serial();
    auto* algorithm_delegate    = AlgorithmDelegate::get_delegate();
    auto  registered_algorithms = algorithm_delegate->get_registered_algorithms();
    auto  os                    = std::ostringstream(std::ios_base::ate);
    os << "{\"count\": " << algorithm_delegate->get_registered_algorithms_count() << ", \"algorithms\": [";
    DELIM_RESET;
    for (const auto& i : registered_algorithms) {
        DELIM_PRINT(os);
        os << "{\"id\": " << unsigned(i.id) << ", \"type\": " << unsigned(i.type)
           << ", \"categroy\": " << unsigned(i.categroy) << ", \"input_type\": " << unsigned(i.input_type) << "}";
    }
    os << "], \"status\": " << int(EL_OK) << ", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_set_algorithm(uint8_t algorithm_id, uint8_t& current_algorithm_id) {
    auto*         serial             = Device::get_device()->get_serial();
    auto*         storage            = DataDelegate::get_delegate()->get_storage_handler();
    auto*         algorithm_delegate = AlgorithmDelegate::get_delegate();
    auto          os                 = std::ostringstream(std::ios_base::ate);
    el_err_code_t ret                = EL_OK;
    if (algorithm_delegate->has_algorithm(algorithm_id)) [[likely]] {
        if (current_algorithm_id != algorithm_id) {
            current_algorithm_id = algorithm_id;
            *storage << el_make_storage_kv("current_algorithm_id", current_algorithm_id);
        }
    } else
        ret = EL_EINVAL;
    os << "{\"algorithm_id\": " << unsigned(algorithm_id) << ", \"status\": " << int(ret)
       << ", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_available_models() {
    auto* serial      = Device::get_device()->get_serial();
    auto* models      = DataDelegate::get_delegate()->get_models_handler();
    auto  models_info = models->get_all_model_info();
    auto  os          = std::ostringstream(std::ios_base::ate);
    os << "{\"count\": " << models->get_all_model_info_size() << ", \"models\": [";
    DELIM_RESET;
    for (const auto& i : models_info) {
        DELIM_PRINT(os);
        os << "{\"id\": " << unsigned(i.id) << ", \"type\": " << unsigned(i.type) << ", \"address\": 0x" << std::hex
           << unsigned(i.addr_flash) << ", \"size\": 0x" << unsigned(i.size) << "}";
    }
    os << std::resetiosflags(std::ios_base::basefield) << "], \"status\": " << int(EL_OK)
       << ", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_set_model(uint8_t model_id, uint8_t& current_model_id, auto* engine) {
    auto*           serial        = Device::get_device()->get_serial();
    auto*           data_delegate = DataDelegate::get_delegate();
    auto*           storage       = data_delegate->get_storage_handler();
    auto*           models        = data_delegate->get_models_handler();
    auto            models_info   = models->get_all_model_info();
    auto            os            = std::ostringstream(std::ios_base::ate);
    el_model_info_t model_info    = models->get_model_info(model_id);
    el_err_code_t   ret           = model_info.id ? EL_OK : EL_EINVAL;
    if (ret != EL_OK) [[unlikely]]
        goto ModelReply;
    // TODO: move heap_caps_malloc to port/el_memory or el_system
    static auto* tensor_arena = heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memset(tensor_arena, 0, kTensorArenaSize);
    ret = engine->init(tensor_arena, kTensorArenaSize);
    if (ret != EL_OK) [[unlikely]]
        goto ModelError;
    ret = engine->load_model(model_info.addr_memory, model_info.size);
    if (ret != EL_OK) [[unlikely]]
        goto ModelError;
    if (current_model_id != model_id) {
        current_model_id = model_id;
        *storage << el_make_storage_kv("current_model_id", current_model_id);
    }
    goto ModelReply;
ModelError:
    current_model_id = 0;
ModelReply:
    os << "{\"model_id\": " << unsigned(model_id) << ", \"status\": " << int(ret)
       << ", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_available_sensors(const auto& registered_sensors) {
    auto*  serial = Device::get_device()->get_serial();
    auto   os     = std::ostringstream(std::ios_base::ate);
    size_t size   = std::distance(registered_sensors.begin(), registered_sensors.end());
    os << "{\"count\": " << size << ", \"sensors\": [";
    DELIM_RESET;
    for (const auto& i : registered_sensors) {
        DELIM_PRINT(os);
        os << "{\"id\": " << unsigned(i.id) << ", \"type\": " << unsigned(i.type)
           << ", \"state\": " << unsigned(i.state) << ", \"parameters\": [";
        DELIM_RESET;
        for (size_t p = 0; p < sizeof(i.parameters); ++p) {
            DELIM_PRINT(os);
            os << unsigned(i.parameters[p]);
        }
        os << "]}";
    }
    os << "], \"status\": " << int(EL_OK) << ", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_set_sensor(uint8_t sensor_id, bool enable, uint8_t& current_sensor_id, auto& registered_sensors) {
    auto* device  = Device::get_device();
    auto* serial  = device->get_serial();
    auto* storage = DataDelegate::get_delegate()->get_storage_handler();
    auto  os      = std::ostringstream(std::ios_base::ate);
    auto  it      = std::find_if(
      registered_sensors.begin(), registered_sensors.end(), [&](const auto& sensor) { return sensor.id == sensor_id; });
    bool          found = it != registered_sensors.end();
    el_err_code_t ret   = found ? EL_OK : EL_EINVAL;
    if (ret != EL_OK) [[unlikely]]
        goto SensorReply;
    // camera
    if (it->type == el_sensor_type_t::SENSOR_TYPE_CAM) {
        auto* camera = device->get_camera();
        it->state    = el_sensor_state_t::SENSOR_STA_LOCKED;
        if (static_cast<bool>(*camera)) ret = camera->deinit();
        if (ret != EL_OK) [[unlikely]]
            goto SensorError;
        if (enable) [[likely]]
            ret = camera->init(it->parameters[0], it->parameters[1]);
        if (ret != EL_OK) [[unlikely]]
            goto SensorError;
        it->state = SENSOR_STA_AVAIL;
        if (current_sensor_id != sensor_id) {
            current_sensor_id = sensor_id;
            *storage << el_make_storage_kv("current_sensor_id", current_sensor_id);
        }
    } else
        ret = EL_ENOTSUP;
    goto SensorReply;
SensorError:
    current_sensor_id = 0;
SensorReply:
    os << "{\"sensor_id\": " << unsigned(sensor_id) << ", \"status\": " << int(ret)
       << ", \"timestamp\": " << el_get_time_ms() << "}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_run_sample(uint8_t sensor_id, const auto& registered_sensors) {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  it     = std::find_if(
      registered_sensors.begin(), registered_sensors.end(), [&](const auto& sensor) { return sensor.id == sensor_id; });
    auto          os    = std::ostringstream(std::ios_base::ate);
    bool          found = it != registered_sensors.end();
    el_err_code_t ret   = found ? EL_OK : EL_EINVAL;
    if (ret != EL_OK) [[unlikely]]
        goto SampleReplyError;
    if (it->state != el_sensor_state_t::SENSOR_STA_AVAIL) [[unlikely]]
        goto SampleReplyError;
    if (it->type == el_sensor_type_t::SENSOR_TYPE_CAM) {
        auto* camera = device->get_camera();
        ret          = camera->start_stream();
        if (ret != EL_OK) [[unlikely]]
            goto SampleReplyError;
        el_img_t img = el_img_t{.data   = nullptr,
                                .size   = 0,
                                .width  = 0,
                                .height = 0,
                                .format = EL_PIXEL_FORMAT_UNKNOWN,
                                .rotate = EL_PIXEL_ROTATE_UNKNOWN};
        ret          = camera->get_frame(&img);
        if (ret != EL_OK) [[unlikely]]
            goto SampleReplyError;
        ret = camera->stop_stream();
        if (ret != EL_OK) [[unlikely]]
            goto SampleReplyError;
        os << "{\"sensor_id\": " << unsigned(it->id) << ", \"type\": " << unsigned(it->type)
           << ", \"status\": " << int(ret) << ", \"data\": \"" << el_img_2_base64_string(&img)
           << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
        goto SampleReply;
    } else
        ret = EL_EINVAL;
SampleReplyError:
    os << "{\"sensor_id\": " << unsigned(sensor_id) << ", \"type\": " << unsigned(found ? unsigned(it->type) : 0u)
       << ", \"status\": " << int(ret) << ", \"size\": 0, \"data\": \"\", \"timestamp\": " << el_get_time_ms() << "}\n";
SampleReply:
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_run_invoke_cls_algorithm(auto*              engine,
                                 int                n_times,
                                 std::atomic<bool>& stop_token,
                                 uint8_t            current_algorithm_id,
                                 uint8_t            current_model_id,
                                 uint8_t            current_sensor_id) {
    auto*         device    = Device::get_device();
    auto*         camera    = device->get_camera();
    auto*         display   = device->get_display();
    auto*         serial    = device->get_serial();
    auto*         img       = new el_img_t{.data   = nullptr,
                                           .size   = 0,
                                           .width  = 0,
                                           .height = 0,
                                           .format = EL_PIXEL_FORMAT_UNKNOWN,
                                           .rotate = EL_PIXEL_ROTATE_UNKNOWN};
    auto*         algorithm = new AlgorithmFOMO(engine);
    el_err_code_t ret       = EL_EIO;
    while ((n_times < 0 || --n_times >= 0) && !stop_token.load()) {
        ret = camera->start_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->get_frame(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->stop_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = algorithm->run(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        display->show(img);
        auto str =
          invoke_results_2_string(algorithm, img, current_algorithm_id, current_model_id, current_sensor_id, ret);
        serial->send_bytes(str.c_str(), str.size());
    }
    if (algorithm) [[likely]]
        delete algorithm;
    if (img) [[likely]]
        delete img;
}

void at_run_invoke_fomo_algorithm(auto*              engine,
                                  int                n_times,
                                  std::atomic<bool>& stop_token,
                                  uint8_t            current_algorithm_id,
                                  uint8_t            current_model_id,
                                  uint8_t            current_sensor_id) {
    auto*         device    = Device::get_device();
    auto*         camera    = device->get_camera();
    auto*         display   = device->get_display();
    auto*         serial    = device->get_serial();
    auto*         img       = new el_img_t{.data   = nullptr,
                                           .size   = 0,
                                           .width  = 0,
                                           .height = 0,
                                           .format = EL_PIXEL_FORMAT_UNKNOWN,
                                           .rotate = EL_PIXEL_ROTATE_UNKNOWN};
    auto*         algorithm = new AlgorithmFOMO(engine);
    el_err_code_t ret       = EL_EIO;
    while ((n_times < 0 || --n_times >= 0) && !stop_token.load()) {
        ret = camera->start_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->get_frame(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->stop_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = algorithm->run(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        uint8_t i = 0;
        for (const auto& box : algorithm->get_results()) {
            int16_t y = box.y - box.h / 2;
            int16_t x = box.x - box.w / 2;
            el_draw_rect(img, x, y, box.w, box.h, color_literal(++i), 4);
        }
        display->show(img);
        auto str =
          invoke_results_2_string(algorithm, img, current_algorithm_id, current_model_id, current_sensor_id, ret);
        serial->send_bytes(str.c_str(), str.size());
    }
    if (algorithm) [[likely]]
        delete algorithm;
    if (img) [[likely]]
        delete img;
}

void at_run_invoke_pfld_algorithm(auto*              engine,
                                  int                n_times,
                                  std::atomic<bool>& stop_token,
                                  uint8_t            current_algorithm_id,
                                  uint8_t            current_model_id,
                                  uint8_t            current_sensor_id) {
    auto*         device    = Device::get_device();
    auto*         camera    = device->get_camera();
    auto*         display   = device->get_display();
    auto*         serial    = device->get_serial();
    auto*         img       = new el_img_t{.data   = nullptr,
                                           .size   = 0,
                                           .width  = 0,
                                           .height = 0,
                                           .format = EL_PIXEL_FORMAT_UNKNOWN,
                                           .rotate = EL_PIXEL_ROTATE_UNKNOWN};
    auto*         algorithm = new AlgorithmFOMO(engine);
    el_err_code_t ret       = EL_EIO;
    while ((n_times < 0 || --n_times >= 0) && !stop_token.load()) {
        ret = camera->start_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->get_frame(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->stop_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = algorithm->run(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        uint8_t i = 0;
        for (const auto& box : algorithm->get_results()) {
            int16_t y = box.y - box.h / 2;
            int16_t x = box.x - box.w / 2;
            el_draw_rect(img, x, y, box.w, box.h, color_literal(++i), 4);
        }
        display->show(img);
        auto str =
          invoke_results_2_string(algorithm, img, current_algorithm_id, current_model_id, current_sensor_id, ret);
        serial->send_bytes(str.c_str(), str.size());
    }
    if (algorithm) [[likely]]
        delete algorithm;
    if (img) [[likely]]
        delete img;
}

void at_run_invoke_yolo_algorithm(auto*              engine,
                                  int                n_times,
                                  std::atomic<bool>& stop_token,
                                  uint8_t            current_algorithm_id,
                                  uint8_t            current_model_id,
                                  uint8_t            current_sensor_id) {
    auto*         device    = Device::get_device();
    auto*         camera    = device->get_camera();
    auto*         display   = device->get_display();
    auto*         serial    = device->get_serial();
    auto*         img       = new el_img_t{.data   = nullptr,
                                           .size   = 0,
                                           .width  = 0,
                                           .height = 0,
                                           .format = EL_PIXEL_FORMAT_UNKNOWN,
                                           .rotate = EL_PIXEL_ROTATE_UNKNOWN};
    auto*         algorithm = new AlgorithmYOLO(engine);
    el_err_code_t ret       = EL_EIO;
    while ((n_times < 0 || --n_times >= 0) && !stop_token.load()) {
        ret = camera->start_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->get_frame(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = camera->stop_stream();
        if (ret != EL_OK) [[unlikely]]
            break;
        ret = algorithm->run(img);
        if (ret != EL_OK) [[unlikely]]
            break;
        uint8_t i = 0;
        for (const auto& point : algorithm->get_results()) el_draw_point(img, point.x, point.y, color_literal(++i));
        display->show(img);
        auto str =
          invoke_results_2_string(algorithm, img, current_algorithm_id, current_model_id, current_sensor_id, ret);
        serial->send_bytes(str.c_str(), str.size());
    }
    if (algorithm) [[likely]]
        delete algorithm;
    if (img) [[likely]]
        delete img;
}

void at_run_invoke(auto*              engine,
                   int                n_times,
                   std::atomic<bool>& stop_token,
                   uint8_t            current_algorithm_id,
                   uint8_t            current_model_id,
                   uint8_t            current_sensor_id,
                   const auto&        registered_sensors) {
    auto*               serial             = Device::get_device()->get_serial();
    auto*               algorithm_delegate = AlgorithmDelegate::get_delegate();
    auto*               models             = DataDelegate::get_delegate()->get_models_handler();
    auto                os                 = std::ostringstream(std::ios_base::ate);
    uint32_t            preprocess_time    = 0;
    uint32_t            run_time           = 0;
    uint32_t            postprocess_time   = 0;
    el_algorithm_info_t algorithm_info{};
    auto sensor_config_it = std::find_if(registered_sensors.begin(), registered_sensors.end(), [&](const auto& sensor) {
        return sensor.id == current_sensor_id && sensor.state == el_sensor_state_t::SENSOR_STA_AVAIL;
    });
    bool is_config_ok     = algorithm_delegate->has_algorithm(current_algorithm_id) &&
                        models->has_model(current_model_id) && (sensor_config_it != registered_sensors.end()) &&
                        [&]() -> bool {
        algorithm_info = algorithm_delegate->get_algorithm(current_algorithm_id);
        return algorithm_info.type == models->get_model_info(current_model_id).type &&
               algorithm_info.input_type == sensor_config_it->type;
    }();
    el_err_code_t ret = is_config_ok ? EL_OK : EL_EINVAL;
    if (ret != EL_OK) [[unlikely]]
        goto InvokeErrorReply;

    switch (algorithm_info.type) {
    case el_algorithm_type_t::ALGORITHM_CLS:
        at_run_invoke_cls_algorithm(
          engine, n_times, stop_token, current_algorithm_id, current_model_id, current_sensor_id);
        break;

    case el_algorithm_type_t::ALGORITHM_FOMO:
        at_run_invoke_fomo_algorithm(
          engine, n_times, stop_token, current_algorithm_id, current_model_id, current_sensor_id);
        break;

    case el_algorithm_type_t::ALGORITHM_PFLD:
        at_run_invoke_pfld_algorithm(
          engine, n_times, stop_token, current_algorithm_id, current_model_id, current_sensor_id);
        break;

    case el_algorithm_type_t::ALGORITHM_YOLO:
        at_run_invoke_yolo_algorithm(
          engine, n_times, stop_token, current_algorithm_id, current_model_id, current_sensor_id);
        break;

    default:
        ret = EL_ENOTSUP;
    }

    return;

InvokeErrorReply:
    os << "{\"algorithm_id\": " << unsigned(current_algorithm_id) << ", \"model_id\": " << unsigned(current_model_id)
       << ", \"sensor_id\": " << unsigned(current_sensor_id) << ", \"status\": " << int(ret)
       << ", \"preprocess_time\": " << unsigned(preprocess_time) << ", \"run_time\": " << unsigned(run_time)
       << ", \"postprocess_time\": " << unsigned(postprocess_time) << ", \"results\": [], \"data\": \"\"}\n";
    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}
