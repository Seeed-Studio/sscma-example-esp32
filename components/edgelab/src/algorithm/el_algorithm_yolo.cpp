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

#include "el_algorithm_yolo.h"

#include "el_cv.h"
#include "el_nms.h"

namespace edgelab {

Yolo::Yolo(InferenceEngine &engine) : Algorithm(engine)
{
    w_scale = 1.0f;
    h_scale = 1.0f;
}

Yolo::~Yolo()
{
}

EL_ERR Yolo::init()
{
    this->results.clear();
    input_shape = this->engine->get_input_shape(0);
    output_shape = this->engine->get_output_shape(0);
    input_quant = this->engine->get_input_quant_param(0);
    output_quant = this->engine->get_output_quant_param(0);
    return EL_OK;
}

EL_ERR Yolo::deinit()
{
    this->results.clear();
    return EL_OK;
}

EL_ERR Yolo::preprocess()
{
    EL_ERR ret = EL_OK;
    el_img_t img = {0};
    el_img_t *i_img = static_cast<el_img_t *>(this->input);

    img.data = static_cast<uint8_t *>(this->engine->get_input(0));
    img.width = input_shape.dims[1];
    img.height = input_shape.dims[2];
    img.size = img.width * img.height * input_shape.dims[3];
    if (input_shape.dims[3] == 3) {
        img.format = EL_PIXEL_FORMAT_RGB888;
    }
    else if (input_shape.dims[3] == 1) {
        img.format = EL_PIXEL_FORMAT_GRAYSCALE;
    }
    else {
        return EL_EINVAL;
    }

    // convert image
    ret = el_img_convert(i_img, &img);
    if (ret != EL_OK) {
        return ret;
    }

    for (int i = 0; i < img.size; i++) {
        img.data[i] -= 128;
    }
    w_scale = float(i_img->width) / float(img.width);
    h_scale = float(i_img->height) / float(img.height);
    return EL_OK;
}

EL_ERR Yolo::postprocess()
{
    this->results.clear();

    // get output
    int8_t *data = static_cast<int8_t *>(this->engine->get_output(0));
    uint16_t width = input_shape.dims[1];
    uint16_t height = input_shape.dims[2];
    float scale = output_quant.scale;
    int32_t zero_point = output_quant.zero_point;
    uint32_t num_record = output_shape.dims[1];
    uint32_t num_element = output_shape.dims[2];
    uint32_t num_class = num_element - 5;

    bool rescale = scale < 0.1 ? true : false;

    // parse output
    for (int i = 0; i < num_record; i++) {
        float score = float(data[i * num_element + INDEX_S] - zero_point) * scale;
        score = rescale ? score * 100 : score;
        if (score > this->score_threshold) {
            el_box_t box;
            int8_t max = -128;
            box.score = score;
            box.target = 0;
            // get box target
            for (uint16_t j = 0; j < num_class; j++) {
                if (max < data[i * num_element + INDEX_T + j]) {
                    max = data[i * num_element + INDEX_T + j];
                    box.target = j;
                }
            }
            // get box position
            float x = float(data[i * num_element + INDEX_X] - zero_point) * scale;
            float y = float(data[i * num_element + INDEX_Y] - zero_point) * scale;
            float w = float(data[i * num_element + INDEX_W] - zero_point) * scale;
            float h = float(data[i * num_element + INDEX_H] - zero_point) * scale;

            if (rescale) {
                box.x = EL_CLIP(uint16_t(x * width), 0, width);
                box.y = EL_CLIP(uint16_t(y * height), 0, height);
                box.w = EL_CLIP(uint16_t(w * width), 0, width);
                box.h = EL_CLIP(uint16_t(h * height), 0, height);
            }
            else {
                box.x = EL_CLIP(uint16_t(x), 0, width);
                box.y = EL_CLIP(uint16_t(y), 0, height);
                box.w = EL_CLIP(uint16_t(w), 0, width);
                box.h = EL_CLIP(uint16_t(h), 0, height);
            }
            box.w = box.x + box.w > width ? width - box.x : box.w;
            box.h = box.y + box.h > height ? height - box.y : box.h;

            this->results.emplace_front(box);
        }
    }

    el_nms(this->results, this->nms_threshold, this->score_threshold);

    // for (auto &box : this->results) {
    //     LOG_D("x: %d, y: %d, w: %d, h: %d, score: %d, target: %d",
    //                box.x,
    //                box.y,
    //                box.w,
    //                box.h,
    //                box.score,
    //                box.target);
    // }
    this->results.sort([](const el_box_t &a, const el_box_t &b) { return a.x < b.x; });
    this->result_size = std::distance(this->results.begin(), this->results.end());

    for (auto &box : this->results) {
        box.x = box.x * w_scale;
        box.y = box.y * h_scale;
        box.w = box.w * w_scale;
        box.h = box.h * h_scale;
    }

    return EL_OK;
}

const void *Yolo::get_result(size_t index)
{
    el_box_t *box = nullptr;
    if (index > this->result_size) {
        return nullptr;
    }
    auto front = this->results.begin();
    std::advance(front, index);
    box = &(*front);
    return box;
}

size_t Yolo::get_result_size()
{
    return this->result_size;
}

} // namespace edgelab
