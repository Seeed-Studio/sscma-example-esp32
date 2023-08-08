

#include <stdio.h>

#include "edgelab.h"

extern "C" void app_main() {
    printf("Data models demo:\n");

    printf("Getting data delegate...\n");
    DataDelegate* data_dalegate = DataDelegate::get();

    printf("Geting models handler from data delegate...\n");
    Models* models = data_dalegate->get_models_handler;

    printf("Init models...\n");
    models->init();

    printf("Get all model info and print ->\n");
    std::forward_list<el_model_info_t> all_model_info = models->get_all_model_info();
    for (const auto& info : all_model_info) 
        printf("\tmodel: {id: %d, type: %d, flash_address: %d, size: %d, memory_address: %p}\n",
            k.id, k.type, k.addr_flash, k.size, k.addr_memory);
    

    printf("Seek all model info again from flash and print ->\n");
    models->seek_models_from_flash();
    std::forward_list<el_model_info_t> all_model_info = models->get_all_model_info();
    for (const auto& i : all_model_info) 
        printf("\tmodel: {id: %d, type: %d, flash_address: %d, size: %d, memory_address: %p}\n",
            i.id, i.type, i.addr_flash, i.size, i.addr_memory);

    printf("Check if has a specified models on the flash ->\n");
    bool has_model = models->has_model(1u);
    if (has_model) {
        el_model_info_t i = models->get_model_info(1u);
        printf("\tmodel: {id: %d, type: %d, flash_address: %d, size: %d, memory_address: %p}\n",
            i.id, i.type, i.addr_flash, i.size, i.addr_memory);
    }

    printf("Get a specified models info ->\n");
    el_model_info_t model_info{};
    el_err_code_t ret = models->get(1u, model_info);
    if (ret == EL_OK) {
        printf("\tmodel: {id: %d, type: %d, flash_address: %d, size: %d, memory_address: %p}\n",
            model_info.id, model_info.type, model_info.addr_flash, model_info.size, model_info.addr_memory);
    }

    // TODO: move freeRTOS, ESP related function call to EdgeLab
    for (int i = 1000; i >= 0; --i) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now...\n");
    fflush(stdout);
    esp_restart();
}
