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

#ifndef _EL_ALGORITHM_FOMO_H_
#define _EL_ALGORITHM_FOMO_H_

#include <cstdint>
#include <type_traits>

#include "el_algorithm_base.hpp"
#include "el_cv.h"
#include "el_debug.h"
#include "el_types.h"

namespace edgelab {
namespace algorithm {

template <typename InferenceEngine, typename ImageType, typename BoxType>
class FOMO : public edgelab::algorithm::base::Algorithm<InferenceEngine, ImageType, BoxType> {
    using ScoreType = edgelab::algorithm::base::Algorithm<InferenceEngine, ImageType, BoxType>::ScoreType;

   public:
    ImageType _input_img;

   protected:
    el_err_code_t preprocess() override;
    el_err_code_t postprocess() override;

   public:
    FOMO(InferenceEngine* engine, ScoreType score_threshold = 80);
    ~FOMO();
};

template <typename InferenceEngine, typename ImageType, typename BoxType>
FOMO<InferenceEngine, ImageType, BoxType>::FOMO(InferenceEngine* engine, ScoreType score_threshold)
    : edgelab::algorithm::base::Algorithm<InferenceEngine, ImageType, BoxType>(engine, score_threshold) {
    _input_img.data   = static_cast<decltype(ImageType::data)>(engine->get_input(0));
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

    // el_registered_algorithms.emplace(
    //   1u, el_algorithm_t{.id = 1, .type = 1, .categroy = 0, .input_type = 0, .parameters = {0, 0, 0, 0}});  // FOMO
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
FOMO<InferenceEngine, ImageType, BoxType>::~FOMO() {
    this->__results.clear();
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
el_err_code_t FOMO<InferenceEngine, ImageType, BoxType>::preprocess() {
    el_err_code_t ret{EL_OK};
    auto*  i_img{this->__p_input};

    // convert image
    el_printf("%d, %d\n", _input_img.width, _input_img.height);
    ret = rgb_to_rgb(i_img, &_input_img);

    if (ret != EL_OK) {
        return ret;
    }

    for (decltype(ImageType::size) i{0}; i < _input_img.size; ++i) {
        _input_img.data[i] -= 128;
    }

    return EL_OK;
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
el_err_code_t FOMO<InferenceEngine, ImageType, BoxType>::postprocess() {
    this->__results.clear();

    // get output
    auto*   data{static_cast<int8_t*>(this->__p_engine->get_output(0))};
    auto    width{this->__p_input->width};
    auto    height{this->__p_input->height};
    float   scale{this->__output_quant.scale};
    int32_t zero_point{this->__output_quant.zero_point};
    auto    pred_w{this->__output_shape.dims[2]};
    auto    pred_h{this->__output_shape.dims[1]};
    auto    pred_t{this->__output_shape.dims[3]};

    auto bw{static_cast<decltype(BoxType::w)>(width / pred_w)};
    auto bh{static_cast<decltype(BoxType::h)>(height / pred_h)};

    for (decltype(pred_h) i{0}; i < pred_h; ++i) {
        for (decltype(pred_w) j{0}; j < pred_w; ++j) {
            ScoreType                 max_score{0};
            decltype(BoxType::target) max_target{0};
            for (decltype(pred_t) t{0}; t < pred_t; ++t) {
                auto score{
                  static_cast<ScoreType>((data[i * pred_w * pred_t + j * pred_t + t] - zero_point) * 100 * scale)};
                if (score > max_score) {
                    max_score  = score;
                    max_target = t;
                }
            }
            if (max_score > this->__score_threshold && max_target != 0) {
                // only unsigned is supported for fast div by 2 (>> 1)
                static_assert(std::is_unsigned<decltype(bw)>::value && std::is_unsigned<decltype(bh)>::value);
                this->__results.emplace_front(BoxType{.x      = static_cast<decltype(BoxType::x)>(j * bw + (bw >> 1)),
                                                      .y      = static_cast<decltype(BoxType::y)>(i * bh + (bh >> 1)),
                                                      .w      = bw,
                                                      .h      = bh,
                                                      .score  = max_score,
                                                      .target = max_target});
            }
        }
    }
    this->__results.sort([](const BoxType& a, const BoxType& b) { return a.x < b.x; });

    return EL_OK;
}

}  // namespace algorithm
}  // namespace edgelab

#endif
