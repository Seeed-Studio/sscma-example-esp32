#include "porting/el_misc.h"
#include "sscma/repl/executor.hpp"

extern "C" void app_main() {
    using namespace sscma;
    using namespace sscma::repl;

    printf("Starting task thread...\n");
    auto* executor = new Executor(2048, 3);
    printf("Task thread started...\n");

    printf("Waiting for tasks...\n");
    el_sleep(1000);

    for (int i = 0; i < 5; ++i) {
        printf("Adding task %d...\n", i);
        executor->add_task([i](auto& stop_token) {
            printf("Task %d executing ->\n\texiting after %d seconds\n", i, 5 - i);
            el_sleep((5 - i) * 1000);
            printf("\ttask %d done ->\n", i);
        });
    }
    printf("Waiting all tasks finished...\n");

    // TODO: move freeRTOS, ESP related function call to EdgeLab
    for (;;) el_sleep(1000);

    delete executor;
}
