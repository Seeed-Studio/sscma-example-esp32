#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdio.h>

#include "edgelab.h"
#include "el_device_esp.h"

extern "C" void app_main(void) {
    Device* device = Device::get_device();
    Serial* serial = device->get_serial();

    serial->init();

    const size_t buffer_size = 128;
    char*        buffer      = new char[buffer_size]{};

    el_printf("\n");
    setvbuf(stdout, NULL, _IONBF, 0);
    fflush(stdout);

    while (true) {
        serial->get_line(buffer, buffer_size);
        serial->send_bytes(buffer, strlen(buffer));
    
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    delete[] buffer;
}
