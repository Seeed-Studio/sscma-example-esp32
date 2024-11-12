#include "ma_transport_spi.h"

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
#include "driver/spi_slave.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST HSPI_HOST
#else
#define RCV_HOST SPI2_HOST
#endif

#define GPIO_MOSI         GPIO_NUM_9
#define GPIO_MISO         GPIO_NUM_8
#define GPIO_SCLK         GPIO_NUM_7
#define GPIO_CS           GPIO_NUM_4

#define _MA_SPI_CMD_PRFX  0x10
#define _MA_SPI_CMD_READ  0x01
#define _MA_SPI_CMD_WRITE 0x02
#define _MA_SPI_CMD_AVAIL 0x03
#define _MA_SPI_CMD_RESET 0x06

namespace ma {

static SPSCRingBuffer<char>* _rb_tx = nullptr;
static SPSCRingBuffer<char>* _rb_rx = nullptr;

static spi_bus_config_t buscfg = {};

static spi_slave_interface_config_t slvcfg = {};

static void slave_task(void*) {
    static spi_slave_transaction_t t = {};
    static const size_t chunk_size   = 2048;
    static char* chunk               = new (std::align_val_t(16)) char[chunk_size];

    while (1) {

        // half-duplex SPI communication

        t.length    = chunk_size * 8;
        t.tx_buffer = nullptr;
        t.rx_buffer = chunk;

        auto rc = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
        if (rc != ESP_OK) {
            MA_LOGE(MA_TAG, "SPI transmit failed %d", rc);
            continue;
        }

        int prefix = chunk[0];

        if (prefix != _MA_SPI_CMD_PRFX) {
            MA_LOGE(MA_TAG, "Invalid prefix: %d", prefix);
            continue;
        }

        int cmd    = chunk[1];
        size_t len = (chunk[2] << 8) | chunk[3];

        switch (cmd) {
            case _MA_SPI_CMD_READ: {
                size_t sent = _rb_tx->pop(chunk, chunk_size > len ? len : chunk_size);
                t.length    = sent * 8;
                t.tx_buffer = chunk;
                t.rx_buffer = nullptr;
                MA_LOGV(MA_TAG, "SPI slave read: %d/%d", sent, len);
                rc = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
                if (rc != ESP_OK) {
                    MA_LOGE(MA_TAG, "SPI transmit failed %d", rc);
                }
            } break;

            case _MA_SPI_CMD_WRITE: {
                char* data = chunk + 4;
                _rb_rx->push(data, len < chunk_size - 4 ? len : chunk_size - 4);

#if MA_LOG_LEVEL >= MA_LOG_LEVEL_VERBOSE
                data[len] = '\0';
                MA_LOGV(MA_TAG, "SPI slave write: %s", data);
#endif

            } break;

            case _MA_SPI_CMD_AVAIL: {
                size_t avail = _rb_tx->size();
                chunk[0]     = avail >> 8;
                chunk[1]     = avail & 0xFF;
                t.length     = 2 * 8;
                t.tx_buffer  = chunk;
                t.rx_buffer  = nullptr;
                MA_LOGV(MA_TAG, "SPI slave avail: %d", avail);
                rc = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
                if (rc != ESP_OK) {
                    MA_LOGE(MA_TAG, "SPI transmit failed %d", rc);
                }
            } break;

            case _MA_SPI_CMD_RESET: {
                _rb_rx->clear();
                _rb_tx->clear();
            } break;

            default:
                MA_LOGE(MA_TAG, "Unknown command: %d", cmd);
                break;
        }
    }
}

static TaskHandle_t spi_task_handle = nullptr;

SPI::SPI() : Transport(MA_TRANSPORT_SPI) {
    buscfg.mosi_io_num   = GPIO_MOSI;
    buscfg.miso_io_num   = GPIO_MISO;
    buscfg.sclk_io_num   = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    slvcfg.mode         = 0;
    slvcfg.spics_io_num = GPIO_CS;
    slvcfg.queue_size   = 3;
    slvcfg.flags        = 0;
}

SPI::~SPI() {
    deInit();
}

ma_err_t SPI::init(const void* config) {
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

    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    if (spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        return MA_FAILED;
    }

    if (xTaskCreate(slave_task, "spi_slave_task", 4096, NULL, 5, &spi_task_handle) != pdPASS) {
        return MA_FAILED;
    }

    m_initialized = true;

    return MA_OK;
}

void SPI::deInit() {
    if (!m_initialized) {
        return;
    }

    spi_slave_free(RCV_HOST);

    vTaskDelete(spi_task_handle);

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

size_t SPI::available() const {
    return _rb_rx->size();
}

size_t SPI::send(const char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_tx->push(data, length);
}

size_t SPI::flush() {
    if (!m_initialized) {
        return 0;
    }
    auto s = _rb_tx->size();
    while (_rb_tx->size()) {
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    return s;
}

size_t SPI::receive(char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_rx->pop(data, length);
}

size_t SPI::receiveIf(char* data, size_t length, char delimiter) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_rx->popIf(data, length, delimiter);
}


}  // namespace ma
