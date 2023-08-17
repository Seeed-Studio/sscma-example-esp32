/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Seeed Technology Co.,Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "el_serial_esp.h"

#include <ctype.h>
#include <driver/usb_serial_jtag.h>

#include "el_debug.h"

namespace edgelab {

SerialEsp::SerialEsp(usb_serial_jtag_driver_config_t driver_config) : _driver_config(driver_config) {}

SerialEsp::~SerialEsp() { deinit(); }

el_err_code_t SerialEsp::init() {
    this->_is_present = usb_serial_jtag_driver_install(&_driver_config) == ESP_OK;

    return this->_is_present ? EL_OK : EL_EIO;
}

el_err_code_t SerialEsp::deinit() {
    this->_is_present = !(usb_serial_jtag_driver_uninstall() == ESP_OK);

    return !this->_is_present ? EL_OK : EL_EIO;
}

char SerialEsp::echo(bool only_visible) {
    EL_ASSERT(this->_is_present);

    char c{get_char()};
    if (only_visible && !isprint(c)) return c;
    send_bytes(&c, sizeof(c));
    return c;
}

char SerialEsp::get_char() {
    EL_ASSERT(this->_is_present);

    char c{'\0'};
    while (!usb_serial_jtag_read_bytes(&c, 1, 15 / portTICK_PERIOD_MS))
        ;
    return c;
}

size_t SerialEsp::get_line(char* buffer, size_t size, const char delim) {
    EL_ASSERT(this->_is_present);

    size_t pos{0};
    char   c{'\0'};
    while (pos < size - 1) {
        if (!usb_serial_jtag_read_bytes(&c, 1, 15 / portTICK_PERIOD_MS)) continue;

        if (c == delim || c == 0x00) [[unlikely]] {
            buffer[pos++] = '\0';
            return pos;
        }
        buffer[pos++] = c;
    }
    buffer[pos++] = '\0';
    return pos;
}

el_err_code_t SerialEsp::read_bytes(char* buffer, size_t size) {
    EL_ASSERT(this->_is_present);

    size_t read{0};
    size_t pos_of_bytes{0};

    while (size) {
        size_t bytes_to_read{size < _driver_config.rx_buffer_size ? size : _driver_config.rx_buffer_size};

        read += usb_serial_jtag_read_bytes(buffer + pos_of_bytes, bytes_to_read, 15 / portTICK_PERIOD_MS);
        pos_of_bytes += bytes_to_read;
        size -= bytes_to_read;
    }

    return read > 0 ? EL_OK : EL_AGAIN;
}

el_err_code_t SerialEsp::send_bytes(const char* buffer, size_t size) {
    EL_ASSERT(this->_is_present);

    size_t sent{0};
    size_t pos_of_bytes{0};

    while (size) {
        size_t bytes_to_send{size < _driver_config.tx_buffer_size ? size : _driver_config.tx_buffer_size};

        sent += usb_serial_jtag_write_bytes(buffer + pos_of_bytes, bytes_to_send, 20 / portTICK_PERIOD_MS);
        pos_of_bytes += bytes_to_send;
        size -= bytes_to_send;
    }

    return sent == pos_of_bytes ? EL_OK : EL_AGAIN;
}

}  // namespace edgelab
