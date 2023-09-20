#include <stdio.h>

#include "core/edgelab.h"
#include "sscma/repl/executor.hpp"

extern "C" void app_main() {
    using namespace sscma;

    auto* executor = new Executor();

    printf("Starting task thread...\n");
    executor->start();
    printf("Task thread started...\n");

    printf("Waiting for tasks...\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 5; ++i) {
        printf("Adding task %d...\n", i);
        executor->add_task([i](auto& stop_token) {
            printf("Task %d executing ->\n\texiting after %d seconds\n", i, 5 - i);
            vTaskDelay((5 - i) * 1000 / portTICK_PERIOD_MS);
            printf("\ttask %d done ->\n", i);
        });
    }
    printf("Waiting all tasks finished...\n");

    // TODO: move freeRTOS, ESP related function call to EdgeLab
    for (;;) vTaskDelay(1000 / portTICK_PERIOD_MS);

    delete executor;
}
