#include "at_callbacks.hpp"
#include "at_utility.hpp"
#include "edgelab.h"
#include "el_device_esp.h"
#include "task_executor.hpp"

extern "C" void app_main(void) {
    // get resource handler and init resources
    auto* device        = Device::get_device();
    auto* display       = device->get_display();
    auto* serial        = device->get_serial();
    auto* instance      = device->get_repl();
    auto* data_delegate = DataDelegate::get_delegate();
    auto* models        = data_delegate->get_models_handler();
    auto* storage       = data_delegate->get_storage_handler();
    auto* engine        = new InferenceEngine<EngineName::TFLite>();
    auto* executor      = new TaskExecutor(CONFIG_PTHREAD_TASK_STACK_SIZE_DEFAULT, CONFIG_PTHREAD_TASK_PRIO_DEFAULT);

    // init resource
    display->init();
    serial->init();
    instance->init();
    models->init();
    storage->init();

    // temporary variables
    int32_t boot_count           = 0;
    uint8_t current_algorithm_id = 0;
    uint8_t current_model_id     = 0;
    uint8_t current_sensor_id    = 0;
    auto    registered_sensors   = std::forward_list<el_sensor_t>({el_sensor_t{.id    = 1,
                                                                               .type  = el_sensor_type_t::SENSOR_TYPE_CAM,
                                                                               .state = el_sensor_state_t::SENSOR_STA_REG,
                                                                               .parameters = {240, 240, 0, 0, 0, 0}}});
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

    // register repl commands
    instance->register_cmd("ID", "Get device ID", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) { at_get_device_id(); });
                               return EL_OK;
                           }));

    instance->register_cmd("NAME", "Get device name", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) { at_get_device_name(); });
                               return EL_OK;
                           }));

    instance->register_cmd(
      "STAT", "Get device status", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          executor->add_task([&](auto& stop_token) {
              at_get_device_status(boot_count, current_algorithm_id, current_model_id, current_sensor_id);
          });
          return EL_OK;
      }));

    instance->register_cmd(
      "VERSION", "Get version details", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          executor->add_task([&](auto& stop_token) { at_get_version(); });
          return EL_OK;
      }));

    instance->register_cmd("RST", "Reboot device", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               executor->add_task([&](auto& stop_token) { device->restart(); });
                               return EL_OK;
                           }));

    instance->register_cmd(
      "ALGO?", "Get available algorithms", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          executor->add_task([&](auto& stop_token) { at_get_available_algorithms(); });
          return EL_OK;
      }));

    instance->register_cmd("ALGO",
                           "Set algorithm for inference by algorithm ID",
                           "ALGO_ID",
                           el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               uint8_t algorithm_id = std::atoi(argv[0]);
                               executor->add_task([&, algorithm_id = std::move(algorithm_id)](auto& stop_token) {
                                   at_set_algorithm(algorithm_id, current_algorithm_id);
                               });
                               return EL_OK;
                           }));

    // TODO: algorithm config command

    instance->register_cmd(
      "MODEL?", "Get available models", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          executor->add_task([&](auto& stop_token) { at_get_available_models(); });
          return EL_OK;
      }));

    instance->register_cmd(
      "MODEL", "Load a model by model ID", "MODEL_ID", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          uint8_t model_id = std::atoi(argv[0]);
          executor->add_task([&, model_id = std::move(model_id)](auto& stop_token) {
              at_set_model(model_id, current_model_id, engine);
          });
          return EL_OK;
      }));

    instance->register_cmd(
      "SENSOR?", "Get available sensors", "", el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          executor->add_task([&](auto& stop_token) { at_get_available_sensors(registered_sensors); });
          return EL_OK;
      }));

    instance->register_cmd("SENSOR",
                           "Set a default sensor by sensor ID",
                           "SENSOR_ID,ENABLE/DISABLE",
                           el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               uint8_t sensor_id = std::atoi(argv[0]);
                               bool    enable    = std::atoi(argv[1]) ? true : false;
                               executor->add_task([&, sensor_id = std::move(sensor_id)](auto& stop_token) {
                                   at_set_sensor(sensor_id, enable, current_sensor_id, registered_sensors);
                               });
                               return EL_OK;
                           }));

    // TODO: sensor config command

    instance->register_cmd(
      "SAMPLE",
      "Sample data from current sensor",
      "SENSOR_ID",
      el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
          uint8_t sensor_id = std::atoi(argv[0]);
          executor->add_task([&](auto& stop_token) { at_run_sample(sensor_id, registered_sensors); });
          return EL_OK;
      }));

    instance->register_cmd("INVOKE",
                           "Invoke for N times (-1 for infinity loop)",
                           "N_TIMES",
                           el_repl_cmd_cb_t([&](int argc, char** argv) -> el_err_code_t {
                               int n_times = std::atoi(argv[0]);
                               executor->add_task([&, n_times = std::move(n_times)](auto& stop_token) mutable {
                                   at_run_invoke(engine,
                                                 n_times,
                                                 stop_token,
                                                 current_algorithm_id,
                                                 current_model_id,
                                                 current_sensor_id,
                                                 registered_sensors);
                               });
                               return EL_OK;
                           }));

    // start task executor
    executor->start();

    // setup components
    {
        using namespace std::string_literals;
        std::string cmd;
        cmd = "AT+ALGO="s + std::to_string(current_algorithm_id) + "\n"s;
        instance->loop(cmd);
        cmd = "AT+MODEL="s + std::to_string(current_model_id) + "\n"s;
        instance->loop(cmd);
        cmd = "AT+SENSOR="s + std::to_string(current_sensor_id) + ",1\n"s;
        instance->loop(cmd);
    }

// enter service pipeline (TODO: pipeline builder)
ServiceLoop:
    instance->loop(serial->get_char());

    goto ServiceLoop;
}
