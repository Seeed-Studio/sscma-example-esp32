#include "ma_transport_serial.h"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

#include "core/utils/ma_ringbuffer.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <driver/usb_serial_jtag.h>

#define _TEST_TXD (GPIO_NUM_43)
#define _TEST_RXD (GPIO_NUM_44)
#define _TEST_RTS (UART_PIN_NO_CHANGE)
#define _TEST_CTS (UART_PIN_NO_CHANGE)

#define _UART_PORT_NUM      (UART_NUM_1)
#define _UART_BAUD_RATE     (921600)
#define _TASK_STACK_SIZE    (4096)

namespace ma {

static auto _driver_config          = usb_serial_jtag_driver_config_t{.tx_buffer_size = 4096 * 4, .rx_buffer_size = 2048};
static SPSCRingBuffer<char>* _rb_rx = nullptr;

static uart_config_t uart_config = {
    .baud_rate  = _UART_BAUD_RATE,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
static int intr_alloc_flags = 0;

Serial::Serial() : Transport(MA_TRANSPORT_SERIAL) {}

Serial::~Serial() {
    deInit();
}

ma_err_t Serial::init(const void* config) {
    if (m_initialized) {
        return MA_OK;
    }

    (void)config;


    if (uart_driver_install(_UART_PORT_NUM, _driver_config.rx_buffer_size , 0, 0, NULL, intr_alloc_flags) != ESP_OK) {
        return MA_EIO;
    }

    if (uart_param_config(_UART_PORT_NUM, &uart_config) != ESP_OK) {
        return MA_EIO;
    }

    if (uart_set_pin(_UART_PORT_NUM, _TEST_TXD, _TEST_RXD, _TEST_RTS, _TEST_CTS) != ESP_OK) {
        return MA_EIO;
    }

    if (_rb_rx == nullptr) {
        _rb_rx = new SPSCRingBuffer<char>(4096);
    }

    m_initialized = true;

    return MA_OK;
}

void Serial::deInit() {
    if (!m_initialized) {
        return;
    }

    uart_driver_delete(_UART_PORT_NUM);

    if (_rb_rx) {
        delete _rb_rx;
        _rb_rx = nullptr;
    }

    m_initialized = false;
}

size_t Serial::available() const {
    return _rb_rx->size();
}

size_t Serial::send(const char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    size_t sent{0};
    size_t pos_of_bytes{0};
    while (length) {
        size_t bytes_to_send{length < _driver_config.tx_buffer_size ? length : _driver_config.tx_buffer_size};

        sent += uart_write_bytes(_UART_PORT_NUM, data + pos_of_bytes, bytes_to_send);
        pos_of_bytes += bytes_to_send;
        length -= bytes_to_send;
    }

    return sent;
}

size_t Serial::flush() {
    if (!m_initialized) {
        return 0;
    }

    return 0;
}

size_t Serial::receive(char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    size_t rlen   = 0;
    char rbuf[32] = {0};  // Most commands are less than 32 bytes long
    do {
        rlen = uart_read_bytes(_UART_PORT_NUM, rbuf, sizeof(rbuf), 10 / portTICK_PERIOD_MS);
        _rb_rx->push(rbuf, rlen);
    } while (rlen > 0);


    return _rb_rx->pop(data, length);
}

size_t Serial::receiveIf(char* data, size_t length, char delimiter) {
    if (!m_initialized) {
        return 0;
    }

    size_t rlen   = 0;
    char rbuf[32] = {0};  // Most commands are less than 32 bytes long
    do {
        rlen = uart_read_bytes(_UART_PORT_NUM, rbuf, sizeof(rbuf), 10 / portTICK_PERIOD_MS);
        for (size_t i = 0; i < rlen; i++) {
           printf("%c", rbuf[i]);
        }
        _rb_rx->push(rbuf, rlen);
    } while (rlen > 0);

    return _rb_rx->popIf(data, length, delimiter);
}


}  // namespace ma
