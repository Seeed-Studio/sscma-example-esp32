#include <algorithm>
#include <atomic>
#include <string>
#include <vector>

#include "at_callbacks.hpp"
#include "at_utility.hpp"
#include "edgelab.h"
#include "el_device_esp.h"

extern "C" void app_main(void) {
    // get resource handler and init resources
    auto* device        = Device::get_device();
    // auto* display       = device->get_display();
    auto* serial        = device->get_serial();
    auto* repl          = ReplDelegate::get_delegate();
    auto* instance      = repl->get_server_handler();
    auto* executor      = repl->get_executor_handler();
    auto* data_delegate = DataDelegate::get_delegate();
    auto* models        = data_delegate->get_models_handler();
    auto* storage       = data_delegate->get_storage_handler();
    auto* engine        = new InferenceEngine();

    // init resource
    // display->init();
    serial->init();
    instance->init(at_server_echo_cb);
    models->init();
    storage->init();

    // temporary variables
    int32_t           boot_count        = 0;
    uint8_t           current_model_id  = 1;
    uint8_t           current_sensor_id = 1;
    std::atomic<bool> is_sample(false);
    std::atomic<bool> is_invoke(false);

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

    // register repl commands (overrite help)
    instance->register_cmd("HELP", "List available commands", "", [&](std::vector<std::string> argv) {
        const auto& registered_cmds = instance->get_registered_cmds();
        at_print_help(registered_cmds);
        return EL_OK;
    });

    instance->register_cmd("ID?", "Get device ID", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_device_id(argv[0]);
                               return EL_OK;
                           }));

    instance->register_cmd("NAME?", "Get device name", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_device_name(argv[0]);
                               return EL_OK;
                           }));

    instance->register_cmd("STAT?", "Get device status", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_device_status(argv[0], boot_count);
                               return EL_OK;
                           }));

    instance->register_cmd("VER?", "Get version details", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_version(argv[0]);
                               return EL_OK;
                           }));

    instance->register_cmd("RST", "Reboot device", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               executor->add_task([&](std::atomic<bool>& stop_token) { device->restart(); });
                               return EL_OK;
                           }));

    instance->register_cmd(
      "BREAK", "Stop all running tasks", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          executor->add_task([cmd = std::string(argv[0])](std::atomic<bool>& stop_token) { at_break(cmd); });
          return EL_OK;
      }));

    instance->register_cmd("YIELD", "Yield for 10ms", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               vTaskDelay(10 / portTICK_PERIOD_MS);
                               return EL_OK;
                           }));

    instance->register_cmd(
      "LED", "Set LED status", "ENABLE/DISABLE", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          el_status_led(std::atoi(argv[1].c_str()) ? true : false);
          return EL_OK;
      }));

    instance->register_cmd(
      "ALGOS?", "Get available algorithms", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_get_available_algorithms(argv[0]);
          return EL_OK;
      }));

    instance->register_cmd("MODELS?", "Get available models", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_available_models(argv[0]);
                               return EL_OK;
                           }));

    instance->register_cmd(
      "MODEL", "Load a model by model ID", "MODEL_ID", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          uint8_t model_id = std::atoi(argv[1].c_str());
          executor->add_task(
            [&, cmd = std::string(argv[0]), model_id = std::move(model_id)](std::atomic<bool>& stop_token) {
                at_set_model(cmd, model_id, engine, current_model_id);
            });
          return EL_OK;
      }));

    instance->register_cmd("MODEL?", "Get current model info", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_model_info(argv[0], engine, current_model_id);
                               return EL_OK;
                           }));

    instance->register_cmd(
      "SENSORS?", "Get available sensors", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_get_available_sensors(argv[0]);
          return EL_OK;
      }));

    instance->register_cmd(
      "SENSOR",
      "Set a default sensor by sensor ID",
      "SENSOR_ID,ENABLE/DISABLE",
      el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          uint8_t sensor_id = std::atoi(argv[1].c_str());
          bool    enable    = std::atoi(argv[2].c_str()) ? true : false;
          executor->add_task(
            [&, cmd = std::string(argv[0]), sensor_id = std::move(sensor_id), enable = std::move(enable)](
              std::atomic<bool>& stop_token) { at_set_sensor(cmd, sensor_id, enable, current_sensor_id); });
          return EL_OK;
      }));

    instance->register_cmd(
      "SENSOR?", "Get current sensor info", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_get_sensor_info(argv[0], current_sensor_id);
          return EL_OK;
      }));

    // TODO: sensor config command

    instance->register_cmd(
      "SAMPLE", "Sample data from current sensor", "N_TIMES", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          int n_times = std::atoi(argv[1].c_str());
          executor->add_task(
            [&, cmd = std::string(argv[0]), n_times = std::move(n_times)](std::atomic<bool>& stop_token) {
                at_run_sample(cmd, n_times, stop_token, current_sensor_id, is_sample);
            });
          return EL_OK;
      }));

    instance->register_cmd(
      "SAMPLE?", "Get sample task status", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_get_task_status(argv[0], is_sample);
          return EL_OK;
      }));

    instance->register_cmd(
      "INVOKE",
      "Invoke for N times (-1 for infinity loop)",
      "N_TIMES,RESULT_ONLY",
      el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          int  n_times     = std::atoi(argv[1].c_str());
          bool result_only = std::atoi(argv[2].c_str()) ? true : false;
          executor->add_task(
            [&, cmd = std::string(argv[0]), n_times = std::move(n_times), result_only = std::move(result_only)](
              std::atomic<bool>& stop_token) mutable {
                at_run_invoke(
                  cmd, n_times, result_only, stop_token, engine, current_model_id, current_sensor_id, is_invoke);
            });
          return EL_OK;
      }));

    instance->register_cmd(
      "INVOKE?", "Get invoke task status", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_get_task_status(argv[0], is_invoke);
          return EL_OK;
      }));

    // Note: AT+ACTION="count(id,0)>=3","LED=1","LED=0"
    instance->register_cmd("ACTION",
                           "Set action trigger",
                           "\"COND\",\"TRUE_CMD\",\"FALSE_OR_EXCEPTION_CMD\"",
                           el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_set_action(argv);
                               return EL_OK;
                           }));

    instance->register_cmd("ACTION!", "Remove action trigger", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_unset_action(argv[0]);
                               return EL_OK;
                           }));

    instance->register_cmd("ACTION?", "Get action trigger", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_get_action(argv[0]);
                               return EL_OK;
                           }));

    instance->register_cmd("INFO",
                           "Store info string to device flash",
                           "\"INFO_STRING\"",
                           el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
                               at_set_info(argv);
                               return EL_OK;
                           }));

    instance->register_cmd(
      "INFO!", "Remove info string from device flash", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_unset_info(argv[0]);
          return EL_OK;
      }));

    instance->register_cmd(
      "INFO?", "Get info string from device flash", "", el_repl_cmd_cb_t([&](std::vector<std::string> argv) {
          at_get_info(argv[0]);
          return EL_OK;
      }));

    // start task executor
    executor->start();

    // setup components
    {
        std::string cmd;
        if (current_model_id) {
            cmd = std::string("AT+MODEL=") + std::to_string(current_model_id);
            instance->exec(cmd);
        }
        if (current_sensor_id) {
            cmd = std::string("AT+SENSOR=") + std::to_string(current_sensor_id) + ",1";
            instance->exec(cmd);
        }
        if (storage->contains("edgelab_action")) {
            char action[CMD_MAX_LENGTH]{};
            *storage >> el_make_storage_kv("edgelab_action", action);
            instance->exec(action_str_2_cmd(action));
        }
        // cmd = std::string("AT+INVOKE=-1,1");
        // instance->exec(cmd);
    }

    // enter service pipeline (TODO: pipeline builder)
    char* buf = new char[CMD_MAX_LENGTH + 1]{};
    for (;;) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        serial->get_line(buf, CMD_MAX_LENGTH);
        instance->exec(buf);
    }
    delete[] buf;

    // release allocated memory (never executed)
    delete engine;
}
