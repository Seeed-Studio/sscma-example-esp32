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
#include "driver/i2c_slave.h"
#include "driver/i2c.h"

#define _MA_I2C_ADDR      0x10
#define _MA_I2C_CMD_READ  0x01
#define _MA_I2C_CMD_WRITE 0x02
#define _MA_I2C_CMD_AVAIL 0x03
#define _MA_I2C_CMD_RESET 0x06

namespace ma {

static SPSCRingBuffer<char>* _rb_tx = nullptr;
static SPSCRingBuffer<char>* _rb_rx = nullptr;

static i2c_slave_dev_handle_t slave_handle;

static i2c_slave_config_t i2c_slave_config = {
    .i2c_port       = I2C_NUM_1,
    .sda_io_num     = GPIO_NUM_5,
    .scl_io_num     = GPIO_NUM_6,
    .clk_source     = I2C_CLK_SRC_DEFAULT,
    .send_buf_depth = 4096 * 4,
    .slave_addr     = _MA_I2C_ADDR,
    .addr_bit_len   = I2C_ADDR_BIT_LEN_7,
    .intr_priority  = 1,
};

static QueueHandle_t slave_recv_queue = nullptr;

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t slave, const i2c_slave_rx_done_event_data_t* edata, void* user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(slave_recv_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static i2c_slave_event_callbacks_t cbs = {
    .on_recv_done = i2c_slave_rx_done_callback,
};

static void slave_task(void* args) {
    static uint8_t* buf = new uint8_t[128];
    static char* chunk  = new char[1024 * 16];

    int rc = ESP_OK;

    while (1) {
        i2c_slave_rx_done_event_data_t rx_data;

        rc = i2c_slave_receive(slave_handle, buf, 128);
        if (rc != ESP_OK) {
            MA_LOGE(MA_TAG, "I2C slave receive failed: %d", rc);
            continue;
        }

        if (pdTRUE == xQueueReceive(slave_recv_queue, &rx_data, portMAX_DELAY)) {
            const uint8_t* data = rx_data.buffer;

            int prefix = data[0];
            int cmd    = data[1];
            int len    = data[2] << 8 | data[3];

            switch (cmd) {
                case _MA_I2C_CMD_READ: {
                    int sent = 0;
                    len      = _rb_tx->size() < len ? _rb_tx->size() : len;
                    while (sent < len) {
                        _rb_tx->pop(chunk, len);
                        rc = i2c_slave_write_buffer((i2c_port_t)i2c_slave_config.i2c_port, reinterpret_cast<uint8_t*>(chunk), len, -1);
                        if (rc != ESP_OK) {
                            MA_LOGE(MA_TAG, "I2C slave write failed: %d", rc);
                        }
                        sent += len;
                    }
                } break;

                case _MA_I2C_CMD_WRITE: {
                    int received = 0;
                    while (received < len) {
                        rc = i2c_slave_read_buffer((i2c_port_t)i2c_slave_config.i2c_port, reinterpret_cast<uint8_t*>(chunk), len, -1);
                        if (rc != ESP_OK) {
                            MA_LOGE(MA_TAG, "I2C slave read failed: %d", rc);
                        }
                        _rb_rx->push(chunk, len);
                        received += len;
                    }
                } break;

                case _MA_I2C_CMD_AVAIL: {
                    int avail = _rb_rx->size();
                    chunk[0]  = avail >> 8;
                    chunk[1]  = avail & 0xFF;
                    rc        = i2c_slave_write_buffer((i2c_port_t)i2c_slave_config.i2c_port, reinterpret_cast<uint8_t*>(chunk), 2, -1);
                    if (rc != ESP_OK) {
                        MA_LOGE(MA_TAG, "I2C slave write failed: %d", rc);
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

    if (slave_recv_queue == nullptr) {
        slave_recv_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    }

    if (_rb_rx == nullptr) {
        _rb_rx = new SPSCRingBuffer<char>(4096);
    }

    if (_rb_tx == nullptr) {
        _rb_tx = new SPSCRingBuffer<char>(4096 * 4);
    }

    if (i2c_new_slave_device(&i2c_slave_config, &slave_handle) != ESP_OK) {
        return MA_FAILED;
    }

    if (i2c_slave_register_event_callbacks(slave_handle, &cbs, slave_recv_queue)) {
        return MA_FAILED;
    }

    if (xTaskCreate(slave_task, "i2c_slave_task", 4096, NULL, 5, &i2c_task_handle) != pdPASS) {
        return MA_FAILED;
    }

    m_initialized = true;

    return MA_OK;
}

void I2C::deInit() {
    if (!m_initialized) {
        return;
    }

    i2c_del_slave_device(slave_handle);

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
