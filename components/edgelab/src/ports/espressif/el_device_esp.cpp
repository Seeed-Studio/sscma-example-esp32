/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Hongtai Liu, nullptr (Seeed Technology Inc.)
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

#include "el_device_esp.h"

#include <esp_efuse.h>
#include <esp_efuse_chip.h>
#include <esp_efuse_table.h>
#include <hal/efuse_hal.h>

#include "el_camera_esp.h"
#include "el_display_esp.h"
#include "el_serial_esp.h"

static inline uint32_t device_id_from_efuse() {
    char*     id_full = new char[16]{};
    esp_err_t err     = esp_efuse_read_field_blob(ESP_EFUSE_OPTIONAL_UNIQUE_ID, id_full, 16 * 8);

    if (err != ESP_OK) return 0ul;

    // Fowler–Noll–Vo hash function
    uint32_t hash  = 0x811c9dc5;
    uint32_t prime = 0x1000193;
    for (size_t i = 0; i < 16; ++i) {
        uint8_t value{id_full[i]};
        hash = hash ^ value;
        hash *= prime;
    }

    return hash;
}

namespace edgelab {

DeviceEsp::DeviceEsp() {
    this->_camera      = static_cast<CameraEsp*>(new CameraEsp());
    this->_display     = static_cast<DisplayEsp*>(new DisplayEsp());
    this->_serial      = static_cast<SerialEsp*>(new SerialEsp());
    this->_repl        = nullptr;
    this->_device_id   = device_id_from_efuse();
    this->_device_name = "Seeed Studio XIAO (ESP32-S3)";
    this->_revision_id = efuse_hal_chip_revision();
}

DeviceEsp::~DeviceEsp() {}

Device* Device::get_device() {
    static DeviceEsp device;
    return &device;
}

void DeviceEsp::restart() { esp_restart(); }

}  // namespace edgelab
