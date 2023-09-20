#include "sscma/main_task.hpp"


extern "C" void app_main(void) {
    sscma::main_task::run();
}
