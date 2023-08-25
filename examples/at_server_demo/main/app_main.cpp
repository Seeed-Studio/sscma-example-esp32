#include <atomic>
#include <string>

#include "at_callbacks.hpp"
#include "at_utility.hpp"
#include "edgelab.h"
#include "el_device_esp.h"

extern "C" void app_main(void) {
    // get resource handler and init resources
    auto* device        = Device::get_device();
    auto* display       = device->get_display();
    auto* serial        = device->get_serial();
    auto* repl          = ReplDelegate::get_delegate();
    auto* instance      = repl->get_server_handler();
    auto* executor      = repl->get_executor_handler();
    auto* data_delegate = DataDelegate::get_delegate();
    auto* models        = data_delegate->get_models_handler();
    auto* storage       = data_delegate->get_storage_handler();
    auto* engine        = new InferenceEngine();

    // init resource
    display->init();
    serial->init();
    instance->init();
    models->init();
    storage->init();

    // temporary variables
    int32_t boot_count        = 0;
    uint8_t current_model_id  = 0;
    uint8_t current_sensor_id = 0;

    // init configs
    if (!storage->contains("edgelab")) {
        *storage << el_make_storage_kv("edgelab", EL_VERSION)
                 << el_make_storage_kv("current_model_id", current_model_id)
                 << el_make_storage_kv("current_sensor_id", current_sensor_id)
                 << el_make_storage_kv("boot_count", boot_count);
    }
    *storage >> el_make_storage_kv("current_model_id", current_model_id) >>
      el_make_storage_kv("current_sensor_id", current_sensor_id) >> el_make_storage_kv("boot_count", boot_count);
    *storage << el_make_storage_kv("boot_count", ++boot_count);

    // register repl commands
    instance->register_cmd(
      "ID?", "Get device ID", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
          executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) { at_get_device_id(cmd); });
          return EL_OK;
      }));

    instance->register_cmd("NAME?", "Get device name", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) {
                                   at_get_device_name(cmd);
                               });
                               return EL_OK;
                           }));

    instance->register_cmd("STAT?", "Get device status", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) {
                                   at_get_device_status(cmd, boot_count, current_model_id, current_sensor_id);
                               });
                               return EL_OK;
                           }));

    instance->register_cmd(
      "VER?", "Get version details", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
          executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) { at_get_version(cmd); });
          return EL_OK;
      }));

    instance->register_cmd("RST", "Reboot device", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&](std::atomic<bool>& stop_token) { device->restart(); });
                               return EL_OK;
                           }));

    instance->register_cmd("BREAK", "Stop all running tasks", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&](std::atomic<bool>& stop_token) {});
                               return EL_OK;
                           }));

    instance->register_cmd("ALGO?", "Get available algorithms", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) {
                                   at_get_available_algorithms(cmd);
                               });
                               return EL_OK;
                           }));

    // // TODO: algorithm config command

    instance->register_cmd("MODEL?", "Get available models", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) {
                                   at_get_available_models(cmd);
                               });
                               return EL_OK;
                           }));

    instance->register_cmd(
      "MODEL", "Load a model by model ID", "MODEL_ID", el_repl_cmd_cb_t([&](int argc, char** argv) {
          uint8_t model_id = std::atoi(argv[1]);
          executor->add_task(
            [&, cmd = std::string(argv[0]), model_id = std::move(model_id)](std::atomic<bool>& stop_token) {
                at_set_model(cmd, model_id, engine, current_model_id);
            });
          return EL_OK;
      }));

    instance->register_cmd("SENSOR?", "Get available sensors", "", el_repl_cmd_cb_t([&](int argc, char** argv) {
                               executor->add_task([&, cmd = std::string(argv[0])](std::atomic<bool>& stop_token) {
                                   at_get_available_sensors(cmd);
                               });
                               return EL_OK;
                           }));

    instance->register_cmd(
      "SENSOR",
      "Set a default sensor by sensor ID",
      "SENSOR_ID,ENABLE/DISABLE",
      el_repl_cmd_cb_t([&](int argc, char** argv) {
          uint8_t sensor_id = std::atoi(argv[1]);
          bool    enable    = std::atoi(argv[2]) ? true : false;
          executor->add_task(
            [&, cmd = std::string(argv[0]), sensor_id = std::move(sensor_id), enable = std::move(enable)](
              std::atomic<bool>& stop_token) { at_set_sensor(cmd, sensor_id, enable, current_sensor_id); });
          return EL_OK;
      }));

    // TODO: sensor config command

    instance->register_cmd(
      "SAMPLE", "Sample data from current sensor", "N_TIMES", el_repl_cmd_cb_t([&](int argc, char** argv) {
          int n_times = std::atoi(argv[1]);
          executor->add_task(
            [&, cmd = std::string(argv[0]), n_times = std::move(n_times)](std::atomic<bool>& stop_token) {
                at_run_sample(cmd, n_times, stop_token, current_sensor_id);
            });
          return EL_OK;
      }));

    instance->register_cmd(
      "INVOKE",
      "Invoke for N times (-1 for infinity loop)",
      "N_TIMES,RESULT_ONLY",
      el_repl_cmd_cb_t([&](int argc, char** argv) {
          int  n_times     = std::atoi(argv[1]);
          bool result_only = std::atoi(argv[2]) ? true : false;
          executor->add_task(
            [&, cmd = std::string(argv[0]), n_times = std::move(n_times), result_only = std::move(result_only)](
              std::atomic<bool>& stop_token) mutable {
                at_run_invoke(cmd, n_times, result_only, stop_token, engine, current_model_id, current_sensor_id);
            });
          return EL_OK;
      }));

    // start task executor
    executor->start();

    // setup components
    {
        auto os = std::ostringstream(std::ios_base::ate);
        os << "AT+MODEL=" << std::to_string(current_model_id) << "\n";
        os << "AT+SENSOR=" << std::to_string(current_sensor_id) << ",1\n";
        instance->loop(os.str());
    }

// enter service pipeline (TODO: pipeline builder)
ServiceLoop:
    instance->loop(serial->get_char());

    goto ServiceLoop;

    // release allocated memory (never executed)
    delete engine;
    engine = nullptr;
}
