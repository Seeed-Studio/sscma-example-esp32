#include <sscma.h>

#include <cstdio>
#include <forward_list>
#include <string>
#include <vector>

#include "ma_codec.h"
#include "ma_server.h"
#include "resource.hpp"


extern "C" void app_main(void) {
    MA_LOGD(MA_TAG, "Initializing Encoder");
    ma::EncoderJSON encoder;

    MA_LOGD(MA_TAG, "Initializing ATServer");
    ma::ATServer server(encoder);

    int ret = 0;

    MA_LOGD(MA_TAG, "Initializing ATServer services");
    ret = server.init();
    if (ret != MA_OK) {
        MA_LOGE(MA_TAG, "ATServer init failed: %d", ret);
    }

    MA_LOGD(MA_TAG, "Starting ATServer");
    ret = server.start();
    if (ret != MA_OK) {
        MA_LOGE(MA_TAG, "ATServer start failed: %d", ret);
    }

    MA_LOGD(MA_TAG, "ATServer started");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
