#include "porting/el_device.h"

extern "C" void app_main(void) {
    using namespace edgelab;

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

        el_sleep(20);
    }

    delete[] buffer;
}
