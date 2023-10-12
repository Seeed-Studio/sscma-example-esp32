
#include <stdio.h>

#include "core/edgelab.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEMO_ESP_WIFI_SSID   "Steins;Gate"
#define DEMO_ESP_WIFI_PASS   "1.048596"

#define DEMO_MQTT_SERVER     "192.168.88.240"
#define DEMO_MQTT_USER       "seeed"
#define DEMO_MQTT_PASS       "xiao_esp32s3"

void demo_mqtt_cb(char* top, int tlen, char* msg, int mlen) {
    printf("GOT MQTT DATA");
    printf("%.*s: %.*s\n", tlen, top, mlen, msg);
}

extern "C" void app_main()
{
    using namespace edgelab;

    Device*  device  = Device::get_device();
    Network* network = device->get_network();

    network->open(DEMO_ESP_WIFI_SSID, DEMO_ESP_WIFI_PASS);
    while (network->status() != NETWORK_READY) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf(".");
    }

    network->connect(DEMO_MQTT_SERVER, DEMO_MQTT_USER, DEMO_MQTT_PASS, demo_mqtt_cb);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for connection established

        char buf[128] = {0};
    for (uint8_t cnt = 0; cnt < 10; ++cnt) {
        sprintf(buf, "hello mqtt %d", cnt);
        network->publish("XIAO/PUB", buf, strlen(buf), MQTT_QOS_0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    network->subscribe("XIAO/SUB", MQTT_QOS_1);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
