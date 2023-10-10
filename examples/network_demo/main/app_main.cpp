
#include <stdio.h>

#include "core/edgelab.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEMO_ESP_WIFI_SSID      "Steins;Gate"
#define DEMO_ESP_WIFI_PASS      "1.048596"

extern "C" void app_main()
{
    using namespace edgelab;

    Device*  device  = Device::get_device();
    Network* network = device->get_network();

    printf("Connecting to %s\n", DEMO_ESP_WIFI_SSID);
    network->open(DEMO_ESP_WIFI_SSID, DEMO_ESP_WIFI_PASS);

    while (true) {
        if (network->status() != WL_CONNECTED)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
