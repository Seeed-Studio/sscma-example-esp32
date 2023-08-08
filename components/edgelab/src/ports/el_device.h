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

#ifndef _EL_DEVICE_H_
#define _EL_DEVICE_H_

#include "el_camera.h"
#include "el_common.h"
#include "el_display.h"
#include "el_repl.h"
#include "el_serial.h"

namespace edgelab {

class Device {
   protected:
    const char* _device_name;
    uint32_t    _device_id;
    uint32_t    _revision_id;

    ReplServer* _repl;
    Camera*     _camera;
    Display*    _display;
    Serial*     _serial;

   public:
    Device(/* args */){};
    ~Device(){};

    ReplServer* get_repl() { return _repl; }
    Camera*     get_camera() { return _camera; }
    Display*    get_display() { return _display; }
    Serial*     get_serial() { return _serial; }
    uint32_t    get_chip_revision_id() { return _revision_id; }
    uint32_t    get_device_id() { return _device_id; }
    const char* get_device_name() { return _device_name; }

    virtual void restart() = 0;

    static Device* get_device();
};

}  // namespace edgelab

#endif /* _EL_Device_H_ */