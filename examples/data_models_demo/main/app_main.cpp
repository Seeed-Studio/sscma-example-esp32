

#include <stdio.h>

#include "core/edgelab.h"

extern "C" void app_main()
{
    printf("Data models demo:\n");

    printf("Geting models handler from data delegate...\n");
    auto* models = new edgelab::Models();

    printf("Init models...\n");
    models->init();

    printf("Get all model info and print ->\n");
    std::forward_list<el_model_info_t> all_model_info = models->get_all_model_info();
    for (const auto& i : all_model_info)
        printf(
            "\tmodel: {id: %d, type: %d, flash_address: 0x%.8lX, size: %ld, memory_address: %p}\n",
            i.id,
            i.type,
            i.addr_flash,
            i.size,
            i.addr_memory);

    printf("Check if has a specified models on the flash ->\n");
    bool has_model = models->has_model(1u);
    if (has_model) {
        el_model_info_t i = models->get_model_info(1u);
        printf(
            "\tmodel: {id: %d, type: %d, flash_address: 0x%.8lX, size: %ld, memory_address: %p}\n",
            i.id,
            i.type,
            i.addr_flash,
            i.size,
            i.addr_memory);
    }

    printf("Get a specified models info ->\n");
    el_model_info_t model_info{};
    el_err_code_t ret = models->get(1u, model_info);
    if (ret == EL_OK) {
        printf(
            "\tmodel: {id: %d, type: %d, flash_address: 0x%.8lX, size: %ld, memory_address: %p}\n",
            model_info.id,
            model_info.type,
            model_info.addr_flash,
            model_info.size,
            model_info.addr_memory);
    }

    printf(
        "Seek all plain TFLite model info again from flash and print (default is packed TFLite "
        "mode) ->\n");
    models->seek_models_from_flash(el_model_format_t::EL_MODEL_FMT_PLAIN_TFLITE);
    all_model_info = models->get_all_model_info();
    for (const auto& i : all_model_info)
        printf(
            "\tmodel: {id: %d, type: %d, flash_address: 0x%.8lX, size: %ld, memory_address: %p}\n",
            i.id,
            i.type,
            i.addr_flash,
            i.size,
            i.addr_memory);

    printf(
        "Seek all packed TFLite model info again from flash and print (faster than plain TFLite "
        "mode) ->\n");
    models->seek_models_from_flash(el_model_format_t::EL_MODEL_FMT_PACKED_TFLITE);
    all_model_info = models->get_all_model_info();
    for (const auto& i : all_model_info)
        printf(
            "\tmodel: {id: %d, type: %d, flash_address: 0x%.8lX, size: %ld, memory_address: %p}\n",
            i.id,
            i.type,
            i.addr_flash,
            i.size,
            i.addr_memory);

    printf("Seek all model info again from flash and print (all models have a unique memory address) ->\n");
    models->seek_models_from_flash(el_model_format_t::EL_MODEL_FMT_PACKED_TFLITE |
                                   el_model_format_t::EL_MODEL_FMT_PLAIN_TFLITE);
    all_model_info = models->get_all_model_info();
    for (const auto& i : all_model_info)
        printf(
            "\tmodel: {id: %d, type: %d, flash_address: 0x%.8lX, size: %ld, memory_address: %p}\n",
            i.id,
            i.type,
            i.addr_flash,
            i.size,
            i.addr_memory);

    // TODO: move freeRTOS, ESP related function call to EdgeLab
    for (int i = 1000; i >= 0; --i) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now...\n");
    fflush(stdout);
    esp_restart();
}
