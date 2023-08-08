
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include "edgelab.h"
#include "el_device_esp.h"


extern "C" void app_main(void) {
    // fetch hardware resource
    auto* device = Device::get_device();
    auto* camera = device->get_camera();
    auto* display = device->get_display();
    auto* serial   = device->get_serial();
    auto* instance = ReplServer::get_instance();

    // fetch resource (TODO: safely delete)
    DataDelegate* datas = DataDelegate::get();
    auto* model_loader{datas->get_models_handler()};
    model_loader->init();
    auto* engine{new InferenceEngine<EngineName::TFLite>()};

    // register algorithms
    register_algorithms();

    // register sensors
    std::unordered_map<uint8_t, el_sensor_t> el_registered_sensors;
    el_registered_sensors.emplace(0u, el_sensor_t{.id = 0, .type = 0, .parameters = {240, 240, 0, 0, 0, 0}});  // camera

    // init persistent map (TODO: move fdb func call to persistent map)
    int32_t        boot_count{0};
    el_algorithm_t default_algorithm{el_registered_algorithms[0]};  // yolo
    el_sensor_t    default_sensor{el_registered_sensors[0]};        // camera
    uint8_t        default_model_id{0};                             // yolo model

    struct fdb_default_kv_node el_default_persistent_map[]{
      {new char[]{"boot_count"}, &boot_count, sizeof(boot_count)},
      {new char[]{"algorithm_config"}, &default_algorithm, sizeof(default_algorithm)},
      {new char[]{"sensor_config"}, &default_sensor, sizeof(default_sensor)},
      {new char[]{"model_id"}, &default_model_id, sizeof(default_model_id)},
    };
    fdb_default_kv default_kv{.kvs = el_default_persistent_map,
                              .num = sizeof(el_default_persistent_map) / sizeof(el_default_persistent_map[0])};
    auto*          persistent_map{datas->get_storage_handler()};
    persistent_map->init(&default_kv);

    // init temporary variables (TODO: current sensors should be a list)
    const el_algorithm_t*     current_algorithm{nullptr};
    const el_model_info_t* current_model{nullptr};
    const el_sensor_t*        current_sensor{nullptr};
    el_img_t*                 current_img{nullptr};

    // fetch configs from persistent map (TODO: value check)
    {
        auto boot_count_kv{el_make_storage_kv("boot_count", boot_count)};
        auto algorithm_config_kv{el_make_storage_kv("algorithm_config", default_algorithm)};
        auto sensor_config_kv{el_make_storage_kv("sensor_config", default_sensor)};
        auto model_id_kv{el_make_storage_kv("model_id", default_model_id)};

        *persistent_map >> boot_count_kv >> algorithm_config_kv >> sensor_config_kv >> model_id_kv;
        *persistent_map << el_make_storage_kv("boot_count", static_cast<decltype(boot_count)>(boot_count_kv.value + 1u));

        boot_count                                             = boot_count_kv.value;
        el_registered_algorithms[algorithm_config_kv.value.id] = algorithm_config_kv.value;
        el_registered_sensors[sensor_config_kv.value.id]       = sensor_config_kv.value;
        current_algorithm = &el_registered_algorithms[algorithm_config_kv.value.id];
        current_model     = &model_loader->get_models()[model_id_kv.value];
        current_sensor    = &el_registered_sensors[sensor_config_kv.value.id];
    }

    // init components (TODO: safely deinit)
    display->init();
    serial->init();
    instance->init();

    // register repl commands
    instance->register_cmd("ID",
                           "Get device ID",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               auto os{std::ostringstream(std::ios_base::ate)};
                               os << "{\"id\": \"" << std::uppercase << std::hex << device->get_device_id()
                                  << std::resetiosflags(std::ios_base::basefield)
                                  << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);
    instance->register_cmd("NAME",
                           "Get device name",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               auto os{std::ostringstream(std::ios_base::ate)};
                               os << "{\"id\": \"" << device->get_device_name()
                                  << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);
    instance->register_cmd("STAT",
                           "Get device status",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               auto os{std::ostringstream(std::ios_base::ate)};
                               os << "{\"boot_count\": " << unsigned(boot_count)
                                  << ", \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);
    instance->register_cmd("VERSION",
                           "Get version details",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               auto os{std::ostringstream(std::ios_base::ate)};
                               os << "{\"edgelab-cpp-sdk\": \"v" << EL_VERSION << "\", \"hardware\": \"v"
                                  << unsigned(device->get_chip_revision_id())
                                  << "\", \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);
    instance->register_cmd("RST",
                           "Reboot device",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               device->restart();
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);
    instance->register_cmd("VALGO",
                           "Get available algorithms",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               auto os{std::ostringstream(std::ios_base::ate)};
                               os << "{\"count\": " << el_registered_algorithms.size() << ", \"algorithms\": [";
                               for (const auto& kv : el_registered_algorithms) {
                                   os << "{\"id\": " << unsigned(kv.second.id)
                                      << ", \"type\": " << unsigned(kv.second.type)
                                      << ", \"categroy\": " << unsigned(kv.second.categroy)
                                      << ", \"input_type\": " << unsigned(kv.second.input_type)
                                      << ", \"parameters\" :[";
                                   for (size_t i{0}; i < sizeof(kv.second.parameters); ++i)
                                       os << unsigned(kv.second.parameters[i]) << ", ";
                                   os << "]}, ";
                               }
                               os << "], \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
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
                               auto algorithm_id{static_cast<uint8_t>(std::atoi(argv[0]))};
                               auto os{std::ostringstream(std::ios_base::ate)};
                               auto it{el_registered_algorithms.find(algorithm_id)};
                               auto ret{it != el_registered_algorithms.end() ? EL_OK : EL_EINVAL};
                               if (ret != EL_OK) [[unlikely]]
                                   goto AlgorithmReply;

                               current_algorithm = &el_registered_algorithms[it->first];
                               *persistent_map
                                 << el_make_storage_kv("algorithm_config", el_registered_algorithms[it->first]);

                           AlgorithmReply:
                               os << "{\"algorithm_id\": " << unsigned(algorithm_id) << ", \"status\": " << int(ret)
                                  << ", \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }));
    // TODO: algorithm config command
    instance->register_cmd(
      "VMODEL",
      "Get available models",
      "",
      el_repl_cmd_exec_cb_t([&]() {
          auto        os{std::ostringstream(std::ios_base::ate)};
          const auto& models{model_loader->get_all_model_info()};
          os << "{\"count\": " << models.size() << ", \"models\": [";
          for (const auto& kv : models)
              os << "{\"id\": " << unsigned(kv.second.id) << ", \"type\": " << unsigned(kv.second.type)
                 << ", \"address\": 0x" << std::hex << unsigned(kv.second.addr_flash) << ", \"size\": 0x"
                 << unsigned(kv.second.size) << "}, ";
          os << std::resetiosflags(std::ios_base::basefield) << "], \"timestamp\": " << el_get_time_ms() << "}\n";
          auto str{os.str()};
          serial->write_bytes(str.c_str(), str.size());
          return EL_OK;
      }),
      nullptr,
      nullptr);
    instance->register_cmd(
      "MODEL",
      "Load a model by model ID",
      "MODEL_ID",
      nullptr,
      nullptr,
      el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
          auto   model_id{static_cast<uint8_t>(std::atoi(argv[0]))};
          auto   os{std::ostringstream(std::ios_base::ate)};
          auto   model{model_loader->get_model_info(model_id)};
          el_err_code_t ret{model.id ? EL_OK : EL_EINVAL};
          if (ret != EL_OK) [[unlikely]]
              goto ModelReply;

          // TODO: move heap_caps_malloc to port/el_memory or el_system
          static auto* tensor_arena{heap_caps_malloc(model.size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)};
          memset(tensor_arena, 0, model.size);
          ret = engine->init(tensor_arena, model.size);
          if (ret != EL_OK) [[unlikely]]
              goto ModelInitError;

          ret = engine->load_model(model.addr_memory, model.size);
          if (ret != EL_OK) [[unlikely]]
              goto ModelInitError;

          current_model = &models[model_id];
          *persistent_map << el_make_storage_kv("model_id", model_id);

          goto ModelReply;

      ModelInitError:
          current_model = nullptr;

      ModelReply:
          os << "{\"model_id\": " << unsigned(model_id) << ", \"status\": " << int(ret)
             << ", \"timestamp\": " << el_get_time_ms() << "}\n";
          auto str{os.str()};
          serial->write_bytes(str.c_str(), str.size());
          return EL_OK;
      }));
    instance->register_cmd("VSENSOR",
                           "Get available sensors",
                           "",
                           el_repl_cmd_exec_cb_t([&]() {
                               auto os{std::ostringstream(std::ios_base::ate)};
                               os << "{\"count\": " << el_registered_sensors.size() << ", \"sensors\": [";
                               for (const auto& kv : el_registered_sensors) {
                                   os << "{\"id\": " << unsigned(kv.second.id)
                                      << ", \"type\": " << unsigned(kv.second.type) << ", \"parameters\" :[";
                                   for (size_t i{0}; i < sizeof(kv.second.parameters); ++i)
                                       os << unsigned(kv.second.parameters[i]) << ", ";
                                   os << "]}, ";
                               }
                               os << "], \"timestamp\": " << el_get_time_ms() << "}\n";
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }),
                           nullptr,
                           nullptr);
    instance->register_cmd(
      "SENSOR",
      "Enable/Disable a sensor by sensor ID",
      "SENSOR_ID,ENABLE_SENSOR",
      nullptr,
      nullptr,
      el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
          if (argc < 2) return EL_EINVAL;
          auto   sensor_id{static_cast<uint8_t>(std::atoi(argv[0]))};
          auto   enable{std::atoi(argv[1]) != 0 ? true : false};
          auto   it{el_registered_sensors.find(sensor_id)};
          auto   os{std::ostringstream(std::ios_base::ate)};
          auto   found{it != el_registered_sensors.end()};
          el_err_code_t ret{found ? EL_OK : EL_EINVAL};
          if (ret != EL_OK) [[unlikely]]
              goto SensorReply;

          // camera
          if (it->second.id == 0 && camera) {
              // camera should be protected to avoid re-init when presented
              ret = enable ? camera->init(it->second.parameters[0], it->second.parameters[1]) : camera->deinit();
              if (ret != EL_OK) [[unlikely]]
                  goto SensorReply;
              if (enable) {
                  current_sensor = &el_registered_sensors[it->first];
                  *persistent_map << el_make_storage_kv("sensor_config", el_registered_sensors[it->first]);
              } else
                  current_sensor = nullptr;
          } else
              ret = EL_ENOTSUP;

      SensorReply:
          os << "{\"sensor_id\": " << unsigned(sensor_id) << ", \"enabled\": " << unsigned(current_sensor ? 1 : 0)
             << ", \"status\": " << int(ret) << ", \"timestamp\": " << el_get_time_ms() << "}\n";
          auto str{os.str()};
          serial->write_bytes(str.c_str(), str.size());
          return EL_OK;
      }));
    // TODO: sensor config command
    instance->register_cmd("SAMPLE",
                           "Sample data from a sensor by sensor ID",
                           "SENSOR_ID,SEND_DATA",
                           nullptr,
                           nullptr,
                           el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               if (argc < 2) return EL_EINVAL;
                               auto   sensor_id{static_cast<uint8_t>(std::atoi(argv[0]))};
                               auto   send_data{std::atoi(argv[1]) != 0 ? true : false};
                               auto   it{el_registered_sensors.find(sensor_id)};
                               auto   os{std::ostringstream(std::ios_base::ate)};
                               auto   found{it != el_registered_sensors.end()};
                               el_err_code_t ret{found ? EL_OK : EL_EINVAL};
                               // TODO: lookup from current sensor list (bit_set<256 - 1>)
                               if (ret != EL_OK) [[unlikely]]
                                   goto SampleReplyError;

                               // camera (TODO: get sensor and allocate buffer from sensor list (bit_set<256 - 1>))
                               if (it->second.type == 0u && camera && static_cast<bool>(*camera)) {
                                   ret = camera->start_stream();
                                   if (ret != EL_OK) [[unlikely]]
                                       goto SampleReplyError;
                                   if (!current_img) [[unlikely]]
                                       current_img = new el_img_t{.data   = nullptr,
                                                                  .size   = 0,
                                                                  .width  = 0,
                                                                  .height = 0,
                                                                  .format = EL_PIXEL_FORMAT_UNKNOWN,
                                                                  .rotate = EL_PIXEL_ROTATE_UNKNOWN};
                                   ret = camera->get_frame(current_img);
                                   if (ret != EL_OK) [[unlikely]]
                                       goto SampleReplyError;
                                   ret = camera->stop_stream();
                                   if (ret != EL_OK) [[unlikely]]
                                       goto SampleReplyError;

                                   os << "{\"sensor_id\": " << unsigned(it->first)
                                      << ", \"type\": " << unsigned(it->second.type) << ", \"status\": " << int(ret)
                                      << ", \"size\": " << unsigned(current_img->size) << ", \"data\": \"";
                                   if (send_data) {
                                       auto size{current_img->width * current_img->height * 3};
                                       auto rgb_img{el_img_t{.data   = new uint8_t[size]{},
                                                             .size   = size,
                                                             .width  = current_img->width,
                                                             .height = current_img->height,
                                                             .format = EL_PIXEL_FORMAT_RGB888,
                                                             .rotate = current_img->rotate}};
                                       ret = rgb_to_rgb(current_img, &rgb_img);
                                       if (ret != EL_OK) {
                                           delete[] rgb_img.data;
                                           goto SampleReplyError;
                                       }
                                       auto jpeg_img{el_img_t{.data   = new uint8_t[size]{},
                                                              .size   = size,
                                                              .width  = rgb_img.width,
                                                              .height = rgb_img.height,
                                                              .format = EL_PIXEL_FORMAT_JPEG,
                                                              .rotate = rgb_img.rotate}};
                                       ret = rgb_to_jpeg(&rgb_img, &jpeg_img);
                                       delete[] rgb_img.data;
                                       if (ret != EL_OK) {
                                           delete[] jpeg_img.data;
                                           goto SampleReplyError;
                                       };

                                       auto buffer{new char[((jpeg_img.size + 2) / 3) * 4]{}};
                                       el_base64_encode(jpeg_img.data, jpeg_img.size, buffer);
                                       delete[] jpeg_img.data;
                                       std::string ss(buffer);
                                       os << ss.c_str();
                                       delete[] buffer;
                                   }
                                   os << "\", \"timestamp\": " << el_get_time_ms() << "}\n";

                                   goto SampleReply;
                               } else
                                   goto SampleReplyError;

                           SampleReplyError:
                               os << "{\"sensor_id\": " << unsigned(sensor_id)
                                  << ", \"type\": " << unsigned(found ? it->second.type : 0xff)
                                  << ", \"status\": " << int(ret)
                                  << ", \"size\": 0, \"data\": \"\", \"timestamp\": " << el_get_time_ms() << "}\n";
                           SampleReply:
                               auto str{os.str()};
                               serial->write_bytes(str.c_str(), str.size());
                               return EL_OK;
                           }));
    instance->register_cmd(
      "INVOKE",
      "Invoke for N times (-1 for infinity loop)",
      "N_TIMES,SEND_DATA",
      nullptr,
      nullptr,
      el_repl_cmd_write_cb_t([&](int argc, char** argv) -> el_err_code_t {
          // TODO: add lock  
          auto thr{std::thread([&]() {
              if (argc < 2) return EL_EINVAL;
              // TODO: N times in a seperate RTOS thread
              auto   n_times{static_cast<int16_t>(std::atoi(argv[0]))};
              auto   os{std::ostringstream(std::ios_base::ate)};
              el_err_code_t ret{current_algorithm ? EL_OK : EL_EINVAL};
              if (ret != EL_OK) [[unlikely]]
                  goto InvokeErrorReply;

              // check if model is loaded
              ret = current_model && current_model->type == current_algorithm->type ? EL_OK : EL_EINVAL;
              if (ret != EL_OK) [[unlikely]]
                  goto InvokeErrorReply;

              // check if sensor is initialized (TODO: find initialized sensor from sensors list/map)
              ret = current_sensor && current_sensor->type == current_algorithm->input_type ? EL_OK : EL_EINVAL;
              if (ret != EL_OK) [[unlikely]]
                  goto InvokeErrorReply;

              // YOLO
              if (current_algorithm->type == 0u) {
                  auto* algorithm{new YOLO(engine)};

                  if (!current_img) [[unlikely]]
                      current_img = new el_img_t{.data   = nullptr,
                                                 .size   = 0,
                                                 .width  = 0,
                                                 .height = 0,
                                                 .format = EL_PIXEL_FORMAT_UNKNOWN,
                                                 .rotate = EL_PIXEL_ROTATE_UNKNOWN};
              InvokeLoop:
                  ret = camera->start_stream();
                  if (ret != EL_OK) [[unlikely]]
                      goto InvokeErrorReply;
                  ret = camera->get_frame(current_img);
                  if (ret != EL_OK) [[unlikely]]
                      goto InvokeErrorReply;
                  ret = camera->stop_stream();
                  if (ret != EL_OK) [[unlikely]]
                      goto InvokeErrorReply;

                  ret = algorithm->run(current_img);
                  if (ret != EL_OK) [[unlikely]] {
                      delete algorithm;
                      goto InvokeErrorReply;
                  }

                  auto preprocess_time{algorithm->get_preprocess_time()};
                  auto run_time{algorithm->get_run_time()};
                  auto postprocess_time{algorithm->get_postprocess_time()};

                  os << "{\"algorithm_id\": " << unsigned(current_algorithm->id)
                     << ", \"type\": " << unsigned(current_algorithm->type) << ", \"status\": " << int(ret)
                     << ", \"preprocess_time\": " << unsigned(preprocess_time)
                     << ", \"run_time\": " << unsigned(run_time)
                     << ", \"postprocess_time\": " << unsigned(postprocess_time) << ", \"results\": ["
                     << el_results_2_string(algorithm->get_results()) << "]}\n";
                  auto str{os.str()};
                  serial->write_bytes(str.c_str(), str.size());
                  if (n_times < 0 || --n_times != 0) {
                      goto InvokeLoop;
                  }

                  delete algorithm;
                  return EL_OK;
              } else
                  ret = EL_ENOTSUP;

          InvokeErrorReply:
              os << "{\"algorithm_id\": " << unsigned(current_algorithm ? current_algorithm->id : 0xff)
                 << ", \"type\": " << unsigned(current_algorithm ? current_algorithm->type : 0xff)
                 << ", \"status\": " << int(ret)
                 << ", \"preprocess_time\": 0, \"run_time\": 0, \"postprocess_time\": 0, \"results\": []}\n";
              auto str{os.str()};
              serial->write_bytes(str.c_str(), str.size());
              return EL_OK;
          })};
          thr.detach();
          return EL_OK;
      }));

    // setup components
    instance->loop("AT+ALGO=0\n", 11);
    instance->loop("AT+MODEL=0\n", 12);
    instance->loop("AT+SENSOR=0,1\n", 15);
    instance->loop("AT+VMODEL\n", 11);

// enter service pipeline (TODO: pipeline builder)
ServiceLoop:
    instance->loop(serial->get_char());

    goto ServiceLoop;
}
