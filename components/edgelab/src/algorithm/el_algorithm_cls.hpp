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

#ifndef _EL_ALGORITHM_CLS_HPP_
#define _EL_ALGORITHM_CLS_HPP_

#include <cstdint>
#include <type_traits>
#include <utility>

#include "el_algorithm_base.hpp"
#include "el_cv.h"
#include "el_debug.h"
#include "el_types.h"

namespace edgelab::algorithm {

namespace types {

// we're not using inheritance since it not standard layout
struct el_algorithm_cls_config_t {
    el_algorithm_info_t info;
};

}  // namespace types

class CLS : public edgelab::algorithm::base::Algorithm {
    using ImageType = el_img_t;
    using ClassType = el_class_t;
    using ScoreType = uint8_t;

   public:
    template <typename InferenceEngine> CLS(InferenceEngine* engine, ScoreType score_threshold = 50);
    ~CLS();

    template <typename InputType> el_err_code_t run(InputType* input);
    const std::forward_list<ClassType>&         get_results() const;

    el_err_code_t set_score_threshold(ScoreType threshold);
    ScoreType     get_score_threshold() const;

   protected:
    el_err_code_t preprocess() override;
    el_err_code_t postprocess() override;

   private:
    ImageType _input_img;
    ScoreType _score_threshold;

    std::forward_list<ClassType> _results;
};

template <typename InferenceEngine>
CLS::CLS(InferenceEngine* engine, ScoreType score_threshold)
    : edgelab::algorithm::base::Algorithm(engine), _score_threshold(score_threshold) {
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

CLS::~CLS() { _results.clear(); }

template <typename InputType> el_err_code_t CLS::run(InputType* input) {
    // TODO: image type conversion before underlying_run, because underlying_run doing a type erasure
    return underlying_run(input);
};

el_err_code_t CLS::preprocess() {
    auto* i_img{static_cast<ImageType*>(this->__p_input)};

    // convert image
    rgb_to_rgb(i_img, &_input_img);

    auto size{_input_img.size};
    for (decltype(ImageType::size) i{0}; i < size; ++i) {
        _input_img.data[i] -= 128;
    }

    return EL_OK;
}

el_err_code_t CLS::postprocess() {
    _results.clear();

    // get output
    auto*   data{static_cast<int8_t*>(this->__p_engine->get_output(0))};
    float   scale{this->__output_quant.scale};
    bool    rescale{scale < 0.1f ? true : false};
    int32_t zero_point{this->__output_quant.zero_point};
    auto    pred_l{this->__output_shape.dims[1]};

    for (decltype(pred_l) i{0}; i < pred_l; ++i) {
        auto score{static_cast<decltype(scale)>(data[i] - zero_point) * scale};
        score = rescale ? score * 100.f : score;
        if (score > _score_threshold)
            _results.emplace_front(ClassType{.score  = static_cast<decltype(ClassType::score)>(score),
                                             .target = static_cast<decltype(ClassType::target)>(i)});
    }
    _results.sort([](const ClassType& a, const ClassType& b) { return a.score > b.score; });

    return EL_OK;
}

const std::forward_list<CLS::ClassType>& CLS::get_results() const { return _results; }

el_err_code_t CLS::set_score_threshold(ScoreType threshold) {
    _score_threshold = threshold;
    return EL_OK;
}

CLS::ScoreType CLS::get_score_threshold() const { return _score_threshold; }

}  // namespace edgelab::algorithm

#endif
