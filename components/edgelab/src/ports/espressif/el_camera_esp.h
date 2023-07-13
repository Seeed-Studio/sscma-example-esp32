/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Hongtai Liu (Seeed Technology Inc.)
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

#ifndef _EL_CAMERA_ESP_H
#define _EL_CAMERA_ESP_H

#include "el_common.h"

#include "el_camera.h"

#include "esp_camera.h"
#include "esp_log.h"

#define XCLK_FREQ_HZ 15000000

namespace edgelab {

class CameraEsp : public Camera {
   private:
    camera_config_t config;
    camera_fb_t *fb;
    sensor_t *sensor;

   public:
    EL_ERR init(size_t width, size_t height) override;
    EL_ERR deinit() override;
    EL_ERR start_stream() override;
    EL_ERR stop_stream() override;
    EL_ERR get_frame(el_img_t *img) override;
    EL_ERR get_jpeg(el_img_t *img) override;
    EL_ERR get_resolutions(el_res_t **res, size_t *res_count) override;

   private:
    framesize_t fit_resolution(size_t width, size_t height);
};

} // namespace edgelab

#endif // EDGELAB_CAMERA_ESP_H_
