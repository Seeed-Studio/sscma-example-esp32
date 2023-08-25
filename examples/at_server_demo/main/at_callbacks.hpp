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

void at_get_device_id(const std::string& cmd) {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": \"" << std::uppercase << std::hex << device->get_device_id()
       << std::resetiosflags(std::ios_base::basefield) << "\"}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_device_name(const std::string& cmd) {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": \"" << device->get_device_name() << "\"}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_device_status(const std::string& cmd,
                          int32_t            boot_count,
                          uint8_t            current_model_id,
                          uint8_t            current_sensor_id) {
    auto* serial = Device::get_device()->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": {\"status\": " << err_code_2_str(EL_OK)
       << ", \"boot_count\": " << static_cast<unsigned>(boot_count)
       << ", \"current_model_id\": " << static_cast<unsigned>(current_model_id)
       << ", \"current_sensor_id\": " << static_cast<unsigned>(current_sensor_id) << "}}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_version(const std::string& cmd) {
    auto* device = Device::get_device();
    auto* serial = device->get_serial();
    auto  os     = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": {\"edgelab_cpp_sdk\": \"" << EL_VERSION << "\", \"chip_revision\": \""
       << static_cast<unsigned>(device->get_chip_revision_id()) << "\"}}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_available_algorithms(const std::string& cmd) {
    auto* serial                = Device::get_device()->get_serial();
    auto* algorithm_delegate    = AlgorithmDelegate::get_delegate();
    auto& registered_algorithms = algorithm_delegate->get_all_algorithm_info();
    auto  os                    = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": [";
    DELIM_RESET;
    for (const auto& i : registered_algorithms) {
        DELIM_PRINT(os);
        os << "{\"type\": " << algo_type_2_str(i->type) << ", \"categroy\": " << algo_cat_2_str(i->categroy)
           << ", \"input_from\": " << sensor_type_2_str(i->input_from) << "}";
    }
    os << "]}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_available_models(const std::string& cmd) {
    auto* serial      = Device::get_device()->get_serial();
    auto* models      = DataDelegate::get_delegate()->get_models_handler();
    auto& models_info = models->get_all_model_info();
    auto  os          = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": [";
    DELIM_RESET;
    for (const auto& i : models_info) {
        DELIM_PRINT(os);
        os << "{\"id\": " << static_cast<unsigned>(i.id) << ", \"type\": " << algo_type_2_str(i.type)
           << ", \"address\": 0x" << std::hex << static_cast<unsigned>(i.addr_flash) << ", \"size\": 0x"
           << static_cast<unsigned>(i.size) << "}" << std::resetiosflags(std::ios_base::basefield);
    }
    os << "]}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_set_model(const std::string& cmd, uint8_t model_id, InferenceEngine* engine, uint8_t& current_model_id) {
    auto* serial        = Device::get_device()->get_serial();
    auto* data_delegate = DataDelegate::get_delegate();
    auto* storage       = data_delegate->get_storage_handler();
    auto* models        = data_delegate->get_models_handler();
    auto  os            = std::ostringstream(std::ios_base::ate);

    el_model_info_t model_info = models->get_model_info(model_id);
    el_err_code_t   ret        = model_info.id ? EL_OK : EL_EINVAL;
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
    os << "\r{\"" << cmd << "\": {\"id\": " << static_cast<unsigned>(model_id)
       << ", \"status\": " << err_code_2_str(ret) << "}}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_get_available_sensors(const std::string& cmd) {
    auto* serial             = Device::get_device()->get_serial();
    auto& registered_sensors = Device::get_device()->get_all_sensor_info();
    auto  os                 = std::ostringstream(std::ios_base::ate);

    os << "\r{\"" << cmd << "\": [";
    DELIM_RESET;
    for (const auto& i : registered_sensors) {
        DELIM_PRINT(os);
        os << "{\"id\": " << static_cast<unsigned>(i.id) << ", \"type\": " << sensor_type_2_str(i.type)
           << ", \"state\": " << sensor_sta_2_str(i.state) << "}";
    }
    os << "]}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_set_sensor(const std::string& cmd, uint8_t sensor_id, bool enable, uint8_t& current_sensor_id) {
    auto* device      = Device::get_device();
    auto* serial      = device->get_serial();
    auto  sensor_info = device->get_sensor_info(sensor_id);
    auto* storage     = DataDelegate::get_delegate()->get_storage_handler();
    auto  os          = std::ostringstream(std::ios_base::ate);

    el_err_code_t ret = sensor_info.id ? EL_OK : EL_EINVAL;
    if (ret != EL_OK) [[unlikely]]
        goto SensorReply;

    // camera
    if (sensor_info.type == el_sensor_type_t::EL_SENSOR_TYPE_CAM) {
        auto* camera = device->get_camera();

        device->set_sensor_state(sensor_id, EL_SENSOR_STA_LOCKED);
        if (static_cast<bool>(*camera)) {
            ret = camera->deinit();
            if (ret != EL_OK) [[unlikely]]
                goto SensorError;
        }

        if (enable) [[likely]] {
            ret = camera->init(240, 240);
            if (ret != EL_OK) [[unlikely]]
                goto SensorError;
        }
        device->set_sensor_state(sensor_id, EL_SENSOR_STA_AVAIL);

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
    os << "\r{\"" << cmd << "\": {\"id\": " << static_cast<unsigned>(sensor_id)
       << ", \"status\": " << err_code_2_str(ret) << "}}\n";

    auto str = os.str();
    serial->send_bytes(str.c_str(), str.size());
}

void at_run_sample(const std::string& cmd, int n_times, std::atomic<bool>& stop_token, uint8_t current_sensor_id) {
    auto* device      = Device::get_device();
    auto* serial      = device->get_serial();
    auto  sensor_info = device->get_sensor_info(current_sensor_id);

    el_err_code_t ret = EL_OK;

    auto direct_reply = [&]() {
        auto os = std::ostringstream(std::ios_base::ate);
        os << "\r{\"" << cmd << "\": {\"status\": " << err_code_2_str(ret)
           << ", \"sensor_id\": " << static_cast<unsigned>(current_sensor_id) << "}}\n";

        auto str = os.str();
        serial->send_bytes(str.c_str(), str.size());
    };
    auto event_reply = [&](const std::string& sample_data_str) {
        auto os = std::ostringstream(std::ios_base::ate);
        os << "\r{\"event\": {\"from\": \"" << cmd << "\", \"status\": " << err_code_2_str(ret) << ", \"contents\": {"
           << sample_data_str << "}}, \"timestamp\": " << el_get_time_ms() << "}\n";

        auto str = os.str();
        serial->send_bytes(str.c_str(), str.size());
    };

    ret = sensor_info.id ? EL_OK : EL_EINVAL;
    if (ret != EL_OK) [[unlikely]]
        goto SampleErrorReply;

    if (sensor_info.state != EL_SENSOR_STA_AVAIL) [[unlikely]]
        goto SampleErrorReply;

    if (sensor_info.type == EL_SENSOR_TYPE_CAM) {
        direct_reply();

        auto*    camera = device->get_camera();
        el_img_t img    = el_img_t{.data   = nullptr,
                                   .size   = 0,
                                   .width  = 0,
                                   .height = 0,
                                   .format = EL_PIXEL_FORMAT_UNKNOWN,
                                   .rotate = EL_PIXEL_ROTATE_UNKNOWN};

        while ((n_times < 0 || --n_times >= 0) && !stop_token.load()) {
            ret = camera->start_stream();
            if (ret != EL_OK) [[unlikely]]
                goto SampleLoopErrorReply;

            ret = camera->get_frame(&img);
            if (ret != EL_OK) [[unlikely]]
                goto SampleLoopErrorReply;

            event_reply(img_2_json_str(&img));

            camera->stop_stream();  // Note: discarding return err_code (always EL_OK)
            continue;

        SampleLoopErrorReply:
            event_reply("");
            break;
        }
        return;
    } else
        ret = EL_EINVAL;

SampleErrorReply:
    direct_reply();
}

template <typename AlgorithmType>
void run_invoke_on_img(AlgorithmType*     algorithm,
                       const std::string& cmd,
                       int                n_times,
                       bool               result_only,
                       std::atomic<bool>& stop_token,
                       uint8_t            model_id,
                       uint8_t            sensor_id) {
    auto*         device      = Device::get_device();
    auto*         camera      = device->get_camera();
    auto*         display     = device->get_display();
    auto*         serial      = device->get_serial();
    auto          img         = el_img_t{.data   = nullptr,
                                         .size   = 0,
                                         .width  = 0,
                                         .height = 0,
                                         .format = EL_PIXEL_FORMAT_UNKNOWN,
                                         .rotate = EL_PIXEL_ROTATE_UNKNOWN};
    el_err_code_t ret         = algorithm ? EL_OK : EL_EINVAL;
    auto          event_reply = [&]() {
        const auto& str = img_invoke_results_2_json_str(algorithm, &img, cmd, result_only, model_id, sensor_id, ret);
        serial->send_bytes(str.c_str(), str.size());
    };
    if (ret != EL_OK) [[unlikely]] {
        event_reply();
        return;
    }

    while ((n_times < 0 || --n_times >= 0) && !stop_token.load()) {
        ret = camera->start_stream();
        if (ret != EL_OK) [[unlikely]]
            goto InvokeErrorReply;

        ret = camera->get_frame(&img);
        if (ret != EL_OK) [[unlikely]]
            goto InvokeErrorReply;

        ret = algorithm->run(&img);
        if (ret != EL_OK) [[unlikely]]
            goto InvokeErrorReply;

        draw_results_on_image(algorithm->get_results(), &img);
        ret = display->show(&img);
        if (ret != EL_OK) [[unlikely]]
            goto InvokeErrorReply;

        event_reply();
        camera->stop_stream();  // Note: discarding return err_code (always EL_OK)
        continue;

    InvokeErrorReply:
        event_reply();
        break;
    }
}

void at_run_invoke(const std::string& cmd,
                   int                n_times,
                   bool               result_only,
                   std::atomic<bool>& stop_token,
                   InferenceEngine*   engine,
                   uint8_t            current_model_id,
                   uint8_t            current_sensor_id) {
    auto* device             = Device::get_device();
    auto* serial             = device->get_serial();
    auto* algorithm_delegate = AlgorithmDelegate::get_delegate();
    auto* data_delegate      = DataDelegate::get_delegate();
    auto* models             = data_delegate->get_models_handler();
    auto* storage            = data_delegate->get_storage_handler();
    auto* instance           = ReplDelegate::get_delegate()->get_server_handler();
    auto  os                 = std::ostringstream(std::ios_base::ate);

    el_model_info_t     model_info{};
    el_algorithm_info_t algorithm_info{};
    el_sensor_info_t    sensor_info{};

    el_err_code_t ret = EL_OK;

    auto direct_reply = [&](const std::string& algorithm_config) {
        os << "\r{\"" << cmd << "\": {\"status\": " << err_code_2_str(ret)
           << ", \"model_id\": " << static_cast<unsigned>(model_info.id)
           << ", \"model_type\": " << algo_type_2_str(model_info.type)
           << ", \"algorithm_category\": " << algo_cat_2_str(algorithm_info.categroy) << ", \"algorithm_config\": {"
           << algorithm_config << "}, \"sensor_id\": " << static_cast<unsigned>(sensor_info.id)
           << ", \"sensor_type\": " << sensor_type_2_str(sensor_info.type)
           << ", \"sensor_state\": " << sensor_sta_2_str(sensor_info.state) << "}}\n";

        auto str = os.str();
        serial->send_bytes(str.c_str(), str.size());
    };

    model_info = models->get_model_info(current_model_id);
    if (model_info.id == 0) [[unlikely]]
        goto InvokeErrorReply;

    algorithm_info = algorithm_delegate->get_algorithm_info(model_info.type);
    if (algorithm_info.type == 0) [[unlikely]]
        goto InvokeErrorReply;

    sensor_info = device->get_sensor_info(current_sensor_id, algorithm_info.input_from);
    if (sensor_info.id == 0 || sensor_info.state != EL_SENSOR_STA_AVAIL) [[unlikely]]
        goto InvokeErrorReply;

    switch (algorithm_info.type) {
    case EL_ALGO_TYPE_IMCLS: {
        auto* algorithm = new AlgorithmIMCLS(engine);

        auto algorithm_config = algorithm->get_algorithm_config();
        auto kv               = el_make_storage_kv_from_type(algorithm_config);
        if (storage->contains(kv.key)) [[likely]]
            *storage >> kv;
        else
            *storage << kv;
        algorithm->set_algorithm_config(kv.value);

        direct_reply(algorithm_config_2_json_str(algorithm_config));

        instance->register_cmd("TSCORE", "Set score threshold", "SCORE_THRESHOLD", [&](int argc, char** argv) {
            kv.value.score_threshold = std::stoi(argv[1]);
            algorithm->set_algorithm_config(kv.value);
            *storage << kv;
            return EL_OK;
        });

        run_invoke_on_img(algorithm, cmd, n_times, result_only, stop_token, current_model_id, current_sensor_id);

        instance->unregister_cmd("TSCORE");
        if (algorithm) [[likely]]
            delete algorithm;
    }
        return;

    case EL_ALGO_TYPE_FOMO: {
        auto* algorithm = new AlgorithmFOMO(engine);

        auto algorithm_config = algorithm->get_algorithm_config();
        auto kv               = el_make_storage_kv_from_type(algorithm_config);
        if (storage->contains(kv.key)) [[likely]]
            *storage >> kv;
        else
            *storage << kv;
        algorithm->set_algorithm_config(kv.value);

        direct_reply(algorithm_config_2_json_str(algorithm_config));

        instance->register_cmd("TSCORE", "Set score threshold", "SCORE_THRESHOLD", [&](int argc, char** argv) {
            kv.value.score_threshold = std::stoi(argv[1]);
            algorithm->set_algorithm_config(kv.value);
            *storage << kv;
            return EL_OK;
        });

        run_invoke_on_img(algorithm, cmd, n_times, result_only, stop_token, current_model_id, current_sensor_id);

        instance->unregister_cmd("TSCORE");
        if (algorithm) [[likely]]
            delete algorithm;
    }
        return;

    case EL_ALGO_TYPE_PFLD: {
        auto* algorithm = new AlgorithmPFLD(engine);

        run_invoke_on_img(algorithm, cmd, n_times, result_only, stop_token, current_model_id, current_sensor_id);

        if (algorithm) [[likely]]
            delete algorithm;
    }
        return;

    case EL_ALGO_TYPE_YOLO: {
        auto* algorithm = new AlgorithmYOLO(engine);

        auto algorithm_config = algorithm->get_algorithm_config();
        auto kv               = el_make_storage_kv_from_type(algorithm_config);
        if (storage->contains(kv.key)) [[likely]]
            *storage >> kv;
        else
            *storage << kv;
        algorithm->set_algorithm_config(kv.value);

        direct_reply(algorithm_config_2_json_str(algorithm_config));

        instance->register_cmd("TSCORE", "Set score threshold", "SCORE_THRESHOLD", [&](int argc, char** argv) {
            kv.value.score_threshold = std::stoi(argv[1]);
            algorithm->set_algorithm_config(kv.value);
            *storage << kv;
            return EL_OK;
        });
        instance->register_cmd("TIOU", "Set IoU threshold", "IOU_THRESHOLD", [&](int argc, char** argv) {
            kv.value.iou_threshold = std::stoi(argv[1]);
            algorithm->set_algorithm_config(kv.value);
            *storage << kv;
            return EL_OK;
        });

        run_invoke_on_img(algorithm, cmd, n_times, result_only, stop_token, current_model_id, current_sensor_id);

        instance->unregister_cmd("TSCORE", "TIOU");
        if (algorithm) [[likely]]
            delete algorithm;
    }
        return;

    default:
        ret = EL_ENOTSUP;
    }

InvokeErrorReply:
    direct_reply("");
}
