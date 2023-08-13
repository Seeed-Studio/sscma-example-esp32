
#include <algorithm>
#include <atomic>
#include <cstring>
#include <forward_list>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include "edgelab.h"
#include "el_device_esp.h"
#include "task_executor.hpp"

#define kTensorArenaSize (1024 * 1024)

uint16_t color[] = {
  0x0000,
  0x03E0,
  0x001F,
  0x7FE0,
  0xFFFF,
};

template <typename AlgorithmType>
std::string invoke_results_2_string(AlgorithmType* algorithm,
                                    el_img_t*      img,
                                    uint8_t        algorithm_id,
                                    uint8_t        model_id,
                                    uint8_t        sensor_id,
                                    el_err_code_t  ret);
std::string el_img_2_base64_string(const el_img_t* img);
const char* el_img_2_base64_c_str(const el_img_t* img);

extern "C" void app_main(void) {
    // get resource handler and init resources
    auto* device             = Device::get_device();
    auto* camera             = device->get_camera();
    auto* display            = device->get_display();
    auto* serial             = device->get_serial();
    auto* data_delegate      = DataDelegate::get_delegate();
    auto* models             = data_delegate->get_models_handler();
    auto* storage            = data_delegate->get_storage_handler();
    auto* algorithm_delegate = AlgorithmDelegate::get_delegate();
    auto* instance           = ReplServer::get_instance();
    auto* engine             = new InferenceEngine<EngineName::TFLite>();

    // init resource (TODO: lazy init thus we can use config parameter to init device)
    camera->init(240, 240);
    display->init();
    serial->init();
    models->init();
    storage->init();
    instance->init();

    // temporary variables
    uint8_t current_algorithm_id  = 0;
    uint8_t current_model_id      = 0;
    uint8_t current_sensor_id     = 0;
    int32_t boot_count            = 0;
    auto    registered_algorithms = algorithm_delegate->get_registered_algorithms();
    auto    registered_sensors =
      std::forward_list<el_sensor_t>({el_sensor_t{.id = 1, .type = 1, .parameters = {240, 240, 0, 0, 0, 0}}});

    // init configs
    if (!storage->contains("edgelab")) {
        *storage << el_make_storage_kv("edgelab", EL_VERSION)
                 << el_make_storage_kv("current_algorithm_id", current_algorithm_id)
                 << el_make_storage_kv("current_model_id", current_model_id)
                 << el_make_storage_kv("current_sensor_id", current_sensor_id)
                 << el_make_storage_kv("boot_count", boot_count);
    }
    *storage >> el_make_storage_kv("current_algorithm_id", current_algorithm_id) >>
      el_make_storage_kv("current_model_id", current_model_id) >>
      el_make_storage_kv("current_sensor_id", current_sensor_id) >> el_make_storage_kv("boot_count", boot_count);
    *storage << el_make_storage_kv("boot_count", ++boot_count);

    // init task executor
    TaskExecutor* executor = new TaskExecutor();

    // register repl commands
    instance->register_cmd("ID",
                           "Get device ID",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) {
                                   auto os = std::ostringstream(std::ios_base::ate);
                                   os << "{\"id\": \"" << std::uppercase << std::hex << device->get_device_id()
                                      << std::resetiosflags(std::ios_base::basefield)
                                      << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("NAME",
                           "Get device name",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) {
                                   auto os = std::ostringstream(std::ios_base::ate);
                                   os << "{\"id\": \"" << device->get_device_name()
                                      << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("STAT",
                           "Get device status",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) {
                                   auto os = std::ostringstream(std::ios_base::ate);
                                   os << "{\"boot_count\": " << unsigned(boot_count)
                                      << ", \"current_algorithm_id\": " << unsigned(current_algorithm_id)
                                      << ", \"current_model_id\": " << unsigned(current_model_id)
                                      << ", \"current_sensor_id\": " << unsigned(current_sensor_id)
                                      << ", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("VERSION",
                           "Get version details",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) {
                                   auto os = std::ostringstream(std::ios_base::ate);
                                   os << "{\"edgelab-cpp-sdk\": \"v" << EL_VERSION << "\", \"hardware\": \"v"
                                      << unsigned(device->get_chip_revision_id())
                                      << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("RST",
                           "Reboot device",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) { device->restart(); });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("VALGO",
                           "Get available algorithms",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) {
                                   auto os = std::ostringstream(std::ios_base::ate);
                                   os << "{\"count\": " << algorithm_delegate->get_registered_algorithms_count()
                                      << ", \"algorithms\": [";
                                   for (const auto& i : registered_algorithms) {
                                       os << "{\"id\": " << unsigned(i.id) << ", \"type\": " << unsigned(i.type)
                                          << ", \"categroy\": " << unsigned(i.categroy)
                                          << ", \"input_type\": " << unsigned(i.input_type) << "}, ";
                                   }
                                   os << "], \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("ALGO",
                           "Set algorithm for inference by algorithm ID",
                           "ALGO_ID",
                           nullptr,
                           nullptr,
                           el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               uint8_t algorithm_id = std::atoi(argv[0]);
                               executor->add_task([&, algorithm_id = std::move(algorithm_id)](auto& stop_token) {
                                   auto          os  = std::ostringstream(std::ios_base::ate);
                                   el_err_code_t ret = EL_OK;
                                   if (algorithm_delegate->has_algorithm(algorithm_id)) [[likely]] {
                                       current_algorithm_id = algorithm_id;
                                       *storage << el_make_storage_kv("current_algorithm_id", current_algorithm_id);
                                   } else
                                       ret = EL_EINVAL;
                                   os << "{\"algorithm_id\": " << unsigned(algorithm_id) << ", \"status\": " << int(ret)
                                      << ", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }));

    // TODO: algorithm config command

    instance->register_cmd(
      "VMODEL",
      "Get available models",
      "",
      el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
          executor->add_task([&](auto& stop_token) {
              auto os          = std::ostringstream(std::ios_base::ate);
              auto models_info = models->get_all_model_info();
              os << "{\"count\": " << models->get_all_model_info_size() << ", \"models\": [";
              for (const auto& i : models_info)
                  os << "{\"id\": " << unsigned(i.id) << ", \"type\": " << unsigned(i.type) << ", \"address\": 0x"
                     << std::hex << unsigned(i.addr_flash) << ", \"size\": 0x" << unsigned(i.size) << "}, ";
              os << std::resetiosflags(std::ios_base::basefield) << "], \"timestamp\": " << el_get_time_ms() << "}\n";
              auto str = os.str();
              serial->write_bytes(str.c_str(), str.size());
          });
          return EL_OK;
      }),
      nullptr,
      nullptr);

    instance->register_cmd("MODEL",
                           "Load a model by model ID",
                           "MODEL_ID",
                           nullptr,
                           nullptr,
                           el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               uint8_t model_id = std::atoi(argv[0]);
                               executor->add_task([&, model_id = std::move(model_id)](auto& stop_token) {
                                   auto            os         = std::ostringstream(std::ios_base::ate);
                                   el_model_info_t model_info = models->get_model_info(model_id);
                                   el_err_code_t   ret        = model_info.id ? EL_OK : EL_EINVAL;
                                   if (ret != EL_OK) [[unlikely]]
                                       goto ModelReply;
                                   // TODO: move heap_caps_malloc to port/el_memory or el_system
                                   static auto* tensor_arena =
                                     heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                                   memset(tensor_arena, 0, kTensorArenaSize);
                                   ret = engine->init(tensor_arena, kTensorArenaSize);
                                   if (ret != EL_OK) [[unlikely]]
                                       goto ModelErrorReply;
                                   ret = engine->load_model(model_info.addr_memory, model_info.size);
                                   if (ret != EL_OK) [[unlikely]]
                                       goto ModelErrorReply;
                                   current_model_id = model_id;
                                   *storage << el_make_storage_kv("current_model_id", current_model_id);
                                   goto ModelReply;
                               ModelErrorReply:
                                   current_model_id = 0;
                               ModelReply:
                                   os << "{\"model_id\": " << unsigned(model_id) << ", \"status\": " << int(ret)
                                      << ", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }));

    instance->register_cmd("VSENSOR",
                           "Get available sensors",
                           "",
                           el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) {
                                   auto   os   = std::ostringstream(std::ios_base::ate);
                                   size_t size = std::distance(registered_sensors.begin(), registered_sensors.end());
                                   os << "{\"count\": " << size << ", \"sensors\": [";
                                   for (const auto& i : registered_sensors) {
                                       os << "{\"id\": " << unsigned(i.id) << ", \"type\": " << unsigned(i.type)
                                          << ", \"parameters\" :[";
                                       for (size_t p = 0; p < sizeof(i.parameters); ++p)
                                           os << unsigned(i.parameters[p]) << ", ";
                                       os << "]}, ";
                                   }
                                   os << "], \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);

    instance->register_cmd("SENSOR",
                           "Set a default sensor by sensor ID",
                           "SENSOR_ID",
                           nullptr,
                           nullptr,
                           el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               uint8_t sensor_id = std::atoi(argv[0]);
                               executor->add_task([&, sensor_id = std::move(sensor_id)](auto& stop_token) {
                                   auto          it    = std::find_if(registered_sensors.begin(),
                                                          registered_sensors.end(),
                                                          [&](const auto& sensor) { return sensor.id == sensor_id; });
                                   auto          os    = std::ostringstream(std::ios_base::ate);
                                   bool          found = it != registered_sensors.end();
                                   el_err_code_t ret   = found ? EL_OK : EL_EINVAL;
                                   if (ret != EL_OK) [[unlikely]]
                                       goto SensorReply;

                                   // camera
                                   if (it->type == 1u && camera && static_cast<bool>(*camera)) {
                                       current_sensor_id = sensor_id;
                                       *storage << el_make_storage_kv("current_sensor_id", current_sensor_id);
                                   } else
                                       ret = EL_ENOTSUP;

                               SensorReply:
                                   os << "{\"sensor_id\": " << unsigned(sensor_id) << ", \"status\": " << int(ret)
                                      << ", \"timestamp\": " << el_get_time_ms() << "}\n";
                                   auto str = os.str();
                                   serial->write_bytes(str.c_str(), str.size());
                               });
                               return EL_OK;
                           }));

    // TODO: sensor config command

    instance->register_cmd(
      "SAMPLE",
      "Sample data from current sensor",
      "",
      el_repl_cmd_exec_cb_t([&]() -> el_err_code_t {
          executor->add_task([&](auto& stop_token) {
              auto it    = std::find_if(registered_sensors.begin(), registered_sensors.end(), [&](const auto& sensor) {
                  return sensor.id == current_sensor_id;
              });
              auto os    = std::ostringstream(std::ios_base::ate);
              bool found = it != registered_sensors.end();
              el_err_code_t ret = found ? EL_OK : EL_EINVAL;
              if (ret != EL_OK) [[unlikely]]
                  goto SampleReplyError;
              if (it->type == 1u && camera && static_cast<bool>(*camera)) {
                  ret = camera->start_stream();
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
              os << "{\"sensor_id\": " << unsigned(current_sensor_id)
                 << ", \"type\": " << unsigned(found ? it->type : 0u) << ", \"status\": " << int(ret)
                 << ", \"size\": 0, \"data\": \"\", \"timestamp\": " << el_get_time_ms() << "}\n";
          SampleReply:
              auto str = os.str();
              serial->write_bytes(str.c_str(), str.size());
          });
          return EL_OK;
      }),
      nullptr,
      nullptr);

    instance->register_cmd(
      "INVOKE",
      "Invoke for N times (-1 for infinity loop)",
      "N_TIMES",
      nullptr,
      nullptr,
      el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
          int n_times = std::atoi(argv[0]);
          executor->add_task([&, n_times = std::move(n_times)](auto& stop_token) mutable {
              auto                os               = std::ostringstream(std::ios_base::ate);
              el_img_t*           img              = nullptr;
              Algorithm*          algorithm        = nullptr;
              uint32_t            preprocess_time  = 0;
              uint32_t            run_time         = 0;
              uint32_t            postprocess_time = 0;
              el_algorithm_info_t algorithm_info{};
              auto                sensor_config_it = std::find_if(registered_sensors.begin(),
                                                   registered_sensors.end(),
                                                   [&](const auto& sensor) { return sensor.id == current_sensor_id; });
              bool                is_config_ok     = algorithm_delegate->has_algorithm(current_algorithm_id) &&
                                  models->has_model(current_model_id) &&
                                  (sensor_config_it != registered_sensors.end()) && [&]() -> bool {
                  algorithm_info = algorithm_delegate->get_algorithm(current_algorithm_id);
                  return algorithm_info.type == models->get_model_info(current_model_id).type &&
                         algorithm_info.input_type == sensor_config_it->type;
              }();
              el_err_code_t ret = is_config_ok ? EL_OK : EL_EINVAL;
              if (ret != EL_OK) [[unlikely]]
                  goto InvokeErrorReply;

              if (!img) [[unlikely]]
                  img = new el_img_t{.data   = nullptr,
                                     .size   = 0,
                                     .width  = 0,
                                     .height = 0,
                                     .format = EL_PIXEL_FORMAT_UNKNOWN,
                                     .rotate = EL_PIXEL_ROTATE_UNKNOWN};

              switch (algorithm_info.type) {
              case el_algorithm_type_t::ALGORITHM_FOMO:
                  if (!algorithm) [[likely]]
                      algorithm = new AlgorithmFOMO(engine);

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

                      ret = static_cast<AlgorithmFOMO*>(algorithm)->run(img);
                      if (ret != EL_OK) [[unlikely]]
                          break;

                      uint8_t i = 0u;
                      for (const auto& box : static_cast<AlgorithmFOMO*>(algorithm)->get_results()) {
                          int16_t y = box.y - box.h / 2;
                          int16_t x = box.x - box.w / 2;
                          el_draw_rect(img, x, y, box.w, box.h, color[++i % 5], 4);
                      }
                      display->show(img);

                      auto str = invoke_results_2_string(static_cast<AlgorithmFOMO*>(algorithm),
                                                         img,
                                                         current_algorithm_id,
                                                         current_model_id,
                                                         current_sensor_id,
                                                         ret);

                      serial->write_bytes(str.c_str(), str.size());
                  }

                  break;

              case el_algorithm_type_t::ALGORITHM_PFLD:
                  if (!algorithm) [[likely]]
                      algorithm = new AlgorithmPFLD(engine);

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

                      ret = static_cast<AlgorithmPFLD*>(algorithm)->run(img);
                      if (ret != EL_OK) [[unlikely]]
                          break;

                      uint8_t i = 0u;
                      for (const auto& point : static_cast<AlgorithmPFLD*>(algorithm)->get_results())
                          el_draw_point(img, point.x, point.y, color[++i % 5]);

                      display->show(img);

                      auto str = invoke_results_2_string(static_cast<AlgorithmPFLD*>(algorithm),
                                                         img,
                                                         current_algorithm_id,
                                                         current_model_id,
                                                         current_sensor_id,
                                                         ret);

                      serial->write_bytes(str.c_str(), str.size());
                  }

                  break;

              case el_algorithm_type_t::ALGORITHM_YOLO:
                  if (!algorithm) [[likely]]
                      algorithm = new AlgorithmYOLO(engine);

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

                      ret = static_cast<AlgorithmYOLO*>(algorithm)->run(img);
                      if (ret != EL_OK) [[unlikely]]
                          break;

                      uint8_t i = 0u;
                      for (const auto& box : static_cast<AlgorithmYOLO*>(algorithm)->get_results()) {
                          int16_t y = box.y - box.h / 2;
                          int16_t x = box.x - box.w / 2;
                          el_draw_rect(img, x, y, box.w, box.h, color[++i % 5], 4);
                      }
                      display->show(img);

                      auto str = invoke_results_2_string(static_cast<AlgorithmYOLO*>(algorithm),
                                                         img,
                                                         current_algorithm_id,
                                                         current_model_id,
                                                         current_sensor_id,
                                                         ret);

                      serial->write_bytes(str.c_str(), str.size());
                  }

                  break;

              default:
                  ret = EL_ENOTSUP;
              }

              if (algorithm) [[likely]]
                  delete algorithm;

              if (img) [[likely]]
                  delete img;

              return;

          InvokeErrorReply:
              os << "{\"algorithm_id\": " << unsigned(current_algorithm_id)
                 << ", \"model_id\": " << unsigned(current_model_id)
                 << ", \"sensor_id\": " << unsigned(current_sensor_id) << ", \"status\": " << int(ret)
                 << ", \"preprocess_time\": " << unsigned(preprocess_time) << ", \"run_time\": " << unsigned(run_time)
                 << ", \"postprocess_time\": " << unsigned(postprocess_time) << ", \"results\": [], \"data\": \"\"}\n";
              auto str = os.str();
              serial->write_bytes(str.c_str(), str.size());
          });

          return EL_OK;
      }));

    // start task executor
    executor->start();

    // setup components (TODO: logic fix, avoid dulpilcate flash write)
    {
        using namespace std::string_literals;
        std::string cmd;
        cmd = "AT+ALGO="s + std::to_string(current_algorithm_id) + "\n"s;
        instance->loop(cmd);
        cmd = "AT+MODEL="s + std::to_string(current_model_id) + "\n"s;
        instance->loop(cmd);
        cmd = "AT+SENSOR="s + std::to_string(current_sensor_id) + "\n"s;
        instance->loop(cmd);
    }

// enter service pipeline (TODO: pipeline builder)
ServiceLoop:
    instance->loop(serial->get_char());

    goto ServiceLoop;

    // release memory (uncalled currently)
    delete executor;
}

template <typename AlgorithmType>
std::string invoke_results_2_string(AlgorithmType* algorithm,
                                    el_img_t*      img,
                                    uint8_t        algorithm_id,
                                    uint8_t        model_id,
                                    uint8_t        sensor_id,
                                    el_err_code_t  ret) {
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

std::string el_img_2_base64_string(const el_img_t* img) {
    if (!img) [[unlikely]]
        return {};
    auto          size    = img->width * img->height * 3;
    auto          rgb_img = el_img_t{.data   = new uint8_t[size]{},
                                     .size   = size,
                                     .width  = img->width,
                                     .height = img->height,
                                     .format = EL_PIXEL_FORMAT_RGB888,
                                     .rotate = img->rotate};
    el_err_code_t ret     = rgb_to_rgb(img, &rgb_img);
    if (ret != EL_OK) [[unlikely]]
        return {};
    auto jpeg_img = el_img_t{.data   = new uint8_t[size]{},
                             .size   = size,
                             .width  = rgb_img.width,
                             .height = rgb_img.height,
                             .format = EL_PIXEL_FORMAT_JPEG,
                             .rotate = rgb_img.rotate};
    ret           = rgb_to_jpeg(&rgb_img, &jpeg_img);
    if (ret != EL_OK) [[unlikely]]
        return {};
    delete[] rgb_img.data;
    auto buffer = new char[((jpeg_img.size + 2) / 3) * 4 + 1]{};
    el_base64_encode(jpeg_img.data, jpeg_img.size, buffer);
    delete[] jpeg_img.data;
    std::string data(buffer);
    delete[] buffer;
    return data;
}

const char* el_img_2_base64_c_str(const el_img_t* img) {
    if (!img) [[unlikely]]
        return {};
    static size_t   size    = 240 * 240 * 3;
    static el_img_t rgb_img = el_img_t{.data   = new uint8_t[size]{},
                                       .size   = size,
                                       .width  = img->width,
                                       .height = img->height,
                                       .format = EL_PIXEL_FORMAT_RGB888,
                                       .rotate = img->rotate};
    el_err_code_t   ret     = rgb_to_rgb(img, &rgb_img);
    if (ret != EL_OK) [[unlikely]]
        return {};
    static el_img_t jpeg_img = el_img_t{.data   = new uint8_t[size]{},
                                        .size   = size,
                                        .width  = rgb_img.width,
                                        .height = rgb_img.height,
                                        .format = EL_PIXEL_FORMAT_JPEG,
                                        .rotate = rgb_img.rotate};
    ret                      = rgb_to_jpeg(&rgb_img, &jpeg_img);
    if (ret != EL_OK) [[unlikely]]
        return {};
    static auto buffer = new char[size]{};
    el_base64_encode(jpeg_img.data, jpeg_img.size, buffer);
    return buffer;
}
