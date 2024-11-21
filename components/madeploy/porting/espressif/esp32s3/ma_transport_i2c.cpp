#include "ma_transport_i2c.h"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

#include "core/ma_debug.h"

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"

#include "core/utils/ma_ringbuffer.hpp"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <sdkconfig.h>

#define _MA_I2C_ADDR      0x62
#define _MA_I2C_CMD_PRFX  0x10
#define _MA_I2C_CMD_READ  0x01
#define _MA_I2C_CMD_WRITE 0x02
#define _MA_I2C_CMD_AVAIL 0x03
#define _MA_I2C_CMD_RESET 0x06

#define _MA_I2C_PORT      I2C_NUM_0

namespace ma {

static SPSCRingBuffer<char>* _rb_tx = nullptr;
static SPSCRingBuffer<char>* _rb_rx = nullptr;

static i2c_config_t i2c_config = {
    .mode          = I2C_MODE_SLAVE,
    .sda_io_num    = GPIO_NUM_5,
    .scl_io_num    = GPIO_NUM_6,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .slave =
        {
            .addr_10bit_en = 0,
            .slave_addr    = _MA_I2C_ADDR,
            .maximum_speed = 0,
        },
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
};

static void slave_task(void* args) {
    static const size_t chunk_size = 1024;
    static char* chunk             = new char[chunk_size + 1];

    while (1) {

        auto sz = i2c_slave_read_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), 1, portMAX_DELAY);
        if (sz != 1) {
            continue;
        }

        int prefix = chunk[0];
        if (prefix != _MA_I2C_CMD_PRFX) {
            continue;
        }

        sz = i2c_slave_read_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), 1, portMAX_DELAY);
        if (sz != 1) {
            continue;
        }

        int cmd = chunk[0];
        switch (cmd) {
            case _MA_I2C_CMD_READ: {
                sz = i2c_slave_read_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), 2, portMAX_DELAY);
                if (sz != 2) {
                    continue;
                }
                size_t len  = (chunk[0] << 8 | chunk[1]) & 0xFFFF;
                size_t sent = _rb_tx->pop(chunk, chunk_size > len ? len : chunk_size);
                MA_LOGV(MA_TAG, "I2C slave read: %d/%d", sent, len);
                sz = i2c_slave_write_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), sent, portMAX_DELAY);
                if (sz < 0) {
                    MA_LOGE(MA_TAG, "I2C slave transmit failed: %d", sz);
                }
            } break;

            case _MA_I2C_CMD_WRITE: {
                sz = i2c_slave_read_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), 2, portMAX_DELAY);
                if (sz != 2) {
                    continue;
                }
                size_t len = (chunk[0] << 8 | chunk[1]) & 0xFFFF;
                len        = len < chunk_size ? len : chunk_size;

                sz = i2c_slave_read_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), len, portMAX_DELAY);
                if (sz != len) {
                    MA_LOGE(MA_TAG, "I2C slave read failed: %d/%d", sz, len);
                    continue;
                }
                _rb_rx->push(chunk, sz);
#if MA_LOG_LEVEL >= MA_LOG_LEVEL_VERBOSE
                chunk[sz] = '\0';
                MA_LOGV(MA_TAG, "I2C slave write: %s", chunk);
#endif
            } break;

            case _MA_I2C_CMD_AVAIL: {
                int avail = _rb_tx->size();
                chunk[0]  = avail >> 8;
                chunk[1]  = avail & 0xFF;
                MA_LOGV(MA_TAG, "I2C slave avail: %d", avail);
                sz = i2c_slave_write_buffer(_MA_I2C_PORT, reinterpret_cast<uint8_t*>(chunk), 2, portMAX_DELAY);
                if (sz < 0) {
                    MA_LOGE(MA_TAG, "I2C slave write failed: %d", sz);
                }
            } break;

            case _MA_I2C_CMD_RESET: {
                _rb_rx->clear();
                _rb_tx->clear();
            } break;

            default:
                MA_LOGE(MA_TAG, "Unknown command: %d", cmd);
        }
    }
}

static TaskHandle_t i2c_task_handle = nullptr;


I2C::I2C() : Transport(MA_TRANSPORT_I2C) {}

I2C::~I2C() {
    deInit();
}

ma_err_t I2C::init(const void* config) {
    if (m_initialized) {
        return MA_OK;
    }

    (void)config;


    if (_rb_rx == nullptr) {
        _rb_rx = new SPSCRingBuffer<char>(4096);
    }

    if (_rb_tx == nullptr) {
        _rb_tx = new SPSCRingBuffer<char>(4096 * 4);
    }

    int rc = i2c_param_config(_MA_I2C_PORT, &i2c_config);
    if (rc != ESP_OK) {
        MA_LOGE(MA_TAG, "Failed to configure I2C slave: %d", rc);
        return MA_FAILED;
    }

    rc = i2c_driver_install(_MA_I2C_PORT, I2C_MODE_SLAVE, 4096, 4096, ESP_INTR_FLAG_IRAM);
    if (rc != ESP_OK) {
        MA_LOGE(MA_TAG, "Failed to install I2C driver: %d", rc);
        return MA_FAILED;
    }

    if (xTaskCreatePinnedToCore(slave_task, "i2c_slave_task", 4096, NULL, 5, &i2c_task_handle, 0) != pdPASS) {
        return MA_FAILED;
    }

    m_initialized = true;

    return MA_OK;
}

void I2C::deInit() {
    if (!m_initialized) {
        return;
    }

    i2c_driver_delete(_MA_I2C_PORT);

    vTaskDelete(i2c_task_handle);

    if (_rb_rx) {
        delete _rb_rx;
        _rb_rx = nullptr;
    }

    if (_rb_tx) {
        delete _rb_tx;
        _rb_tx = nullptr;
    }

    m_initialized = false;
}

size_t I2C::available() const {
    return _rb_rx->size();
}

size_t I2C::send(const char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_tx->push(data, length);
}

size_t I2C::flush() {
    if (!m_initialized) {
        return 0;
    }
    auto s = _rb_tx->size();
    while (_rb_tx->size()) {
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    return s;
}

size_t I2C::receive(char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_rx->pop(data, length);
}

size_t I2C::receiveIf(char* data, size_t length, char delimiter) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_rx->popIf(data, length, delimiter);
}


}  // namespace ma
