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

#ifndef _EL_ALGORITHM_YOLO_HPP_
#define _EL_ALGORITHM_YOLO_HPP_

#include <cstdint>
#include <type_traits>
#include <utility>

#include "el_algorithm_base.hpp"
#include "el_cv.h"
#include "el_debug.h"
#include "el_nms.h"
#include "el_types.h"

namespace edgelab::algorithm {

namespace types {

// we're not using inheritance since it not standard layout
struct el_algorithm_yolo_config_t {
    el_algorithm_info_t info;
    uint8_t             score_threshold;
    uint8_t             nms_threshold;
};

}  // namespace types

class YOLO : public edgelab::algorithm::base::Algorithm {
    using ImageType        = el_img_t;
    using BoxType          = el_box_t;
    using ScoreType        = uint8_t;
    using NMSThresholdType = uint8_t;

   public:
    template <typename InferenceEngine>
    YOLO(InferenceEngine* engine, ScoreType score_threshold = 50, NMSThresholdType nms_threshold = 45);
    ~YOLO();

    template <typename InputType> el_err_code_t run(InputType* input);
    const std::forward_list<BoxType>&           get_results() const;

    el_err_code_t set_score_threshold(ScoreType threshold);
    ScoreType     get_score_threshold() const;

    el_err_code_t    set_nms_threshold(ScoreType threshold);
    NMSThresholdType get_nms_threshold() const;

   protected:
    el_err_code_t preprocess() override;
    el_err_code_t postprocess() override;

   private:
    enum {
        INDEX_X = 0,
        INDEX_Y = 1,
        INDEX_W = 2,
        INDEX_H = 3,
        INDEX_S = 4,
        INDEX_T = 5,
    };

    ImageType        _input_img;
    float            _w_scale;
    float            _h_scale;
    ScoreType        _score_threshold;
    NMSThresholdType _nms_threshold;

    std::forward_list<BoxType> _results;
};

template <typename InferenceEngine>
YOLO::YOLO(InferenceEngine* engine, ScoreType score_threshold, NMSThresholdType nms_threshold)
    : edgelab::algorithm::base::Algorithm(engine),
      _w_scale(1.f),
      _h_scale(1.f),
      _score_threshold(score_threshold),
      _nms_threshold(nms_threshold) {
    _input_img.data   = static_cast<decltype(ImageType::data)>(this->__p_engine->get_input(0));
    _input_img.width  = static_cast<decltype(ImageType::width)>(this->__input_shape.dims[1]),
    _input_img.height = static_cast<decltype(ImageType::height)>(this->__input_shape.dims[2]),
    _input_img.size =
      static_cast<decltype(ImageType::size)>(_input_img.width * _input_img.height * this->__input_shape.dims[3]);
    _input_img.format = EL_PIXEL_FORMAT_UNKNOWN;
    _input_img.rotate = EL_PIXEL_ROTATE_0;

    if (this->__input_shape.dims[3] == 3) {
        _input_img.format = EL_PIXEL_FORMAT_RGB888;
    } else if (this->__input_shape.dims[3] == 1) {
        _input_img.format = EL_PIXEL_FORMAT_GRAYSCALE;
    }

    EL_ASSERT(_input_img.format != EL_PIXEL_FORMAT_UNKNOWN);
    EL_ASSERT(_input_img.rotate != EL_PIXEL_ROTATE_UNKNOWN);
}

YOLO::~YOLO() {
    _results.clear();
    this->__p_engine = nullptr;
}

template <typename InputType> el_err_code_t YOLO::run(InputType* input) {
    _w_scale = static_cast<float>(input->width) / static_cast<float>(_input_img.width);
    _h_scale = static_cast<float>(input->height) / static_cast<float>(_input_img.height);

    // TODO: image type conversion before underlying_run, because underlying_run doing a type erasure
    return underlying_run(input);
};

el_err_code_t YOLO::preprocess() {
    auto* i_img{static_cast<ImageType*>(this->__p_input)};

    // convert image
    rgb_to_rgb(i_img, &_input_img);

    auto size{_input_img.size};
    for (decltype(ImageType::size) i{0}; i < size; ++i) {
        _input_img.data[i] -= 128;
    }

    return EL_OK;
}

el_err_code_t YOLO::postprocess() {
    _results.clear();

    // get output
    auto*   data{static_cast<int8_t*>(this->__p_engine->get_output(0))};
    auto    width{this->__input_shape.dims[1]};
    auto    height{this->__input_shape.dims[2]};
    float   scale{this->__output_quant.scale};
    bool    rescale{scale < 0.1f ? true : false};
    int32_t zero_point{this->__output_quant.zero_point};
    auto    num_record{this->__output_shape.dims[1]};
    auto    num_element{this->__output_shape.dims[2]};
    auto    num_class{static_cast<uint8_t>(num_element - 5)};

    // parse output
    for (decltype(num_record) i{0}; i < num_record; ++i) {
        auto idx{i * num_element};
        auto score{static_cast<decltype(scale)>(data[idx + INDEX_S] - zero_point) * scale};
        score = rescale ? score * 100.f : score;
        if (score > _score_threshold) {
            BoxType box{
              .x      = 0,
              .y      = 0,
              .w      = 0,
              .h      = 0,
              .score  = static_cast<decltype(BoxType::score)>(score),
              .target = 0,
            };

            // get box target
            int8_t max{-128};
            for (decltype(num_class) t{0}; t < num_class; ++t) {
                if (max < data[idx + INDEX_T + t]) {
                    max        = data[idx + INDEX_T + t];
                    box.target = t;
                }
            }

            // get box position, int8_t - int32_t (narrowing)
            auto x{static_cast<decltype(BoxType::x)>((data[idx + INDEX_X] - zero_point) * scale)};
            auto y{static_cast<decltype(BoxType::y)>((data[idx + INDEX_Y] - zero_point) * scale)};
            auto w{static_cast<decltype(BoxType::w)>((data[idx + INDEX_W] - zero_point) * scale)};
            auto h{static_cast<decltype(BoxType::h)>((data[idx + INDEX_H] - zero_point) * scale)};

            box.x = EL_CLIP(x, 0, width) * _w_scale;
            box.y = EL_CLIP(y, 0, height) * _h_scale;
            box.w = EL_CLIP(w, 0, width) * _w_scale;
            box.h = EL_CLIP(h, 0, height) * _h_scale;

            _results.emplace_front(std::move(box));
        }
    }
    el_nms(_results, _nms_threshold, _score_threshold, false, true);

    _results.sort([](const BoxType& a, const BoxType& b) { return a.x < b.x; });

    return EL_OK;
}

const std::forward_list<YOLO::BoxType>& YOLO::get_results() const { return _results; }

el_err_code_t YOLO::set_score_threshold(ScoreType threshold) {
    _score_threshold = threshold;
    return EL_OK;
}

YOLO::ScoreType YOLO::get_score_threshold() const { return _score_threshold; }

el_err_code_t YOLO::set_nms_threshold(NMSThresholdType threshold) {
    _nms_threshold = threshold;
    return EL_OK;
}

YOLO::NMSThresholdType YOLO::get_nms_threshold() const { return _nms_threshold; }

}  // namespace edgelab::algorithm

#endif
