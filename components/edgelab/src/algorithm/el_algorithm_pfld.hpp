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

#ifndef _EL_ALGORITHM_PFLD_HPP_
#define _EL_ALGORITHM_PFLD_HPP_

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
struct el_algorithm_pfld_config_t {
    el_algorithm_info_t info;
};

}  // namespace types

class PFLD : public edgelab::algorithm::base::Algorithm {
    using ImageType = el_img_t;
    using PointType = el_point_t;

   public:
    template <typename InferenceEngine> PFLD(InferenceEngine* engine);
    ~PFLD();

    template <typename InputType> el_err_code_t run(InputType* input);
    const std::forward_list<PointType>&         get_results() const;

   protected:
    el_err_code_t preprocess() override;
    el_err_code_t postprocess() override;

   private:
    ImageType _input_img;

    std::forward_list<PointType> _results;
};

template <typename InferenceEngine> PFLD::PFLD(InferenceEngine* engine) : edgelab::algorithm::base::Algorithm(engine) {
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
}

PFLD::~PFLD() { _results.clear(); }

template <typename InputType> el_err_code_t PFLD::run(InputType* input) {
    // TODO: image type conversion before underlying_run, because underlying_run doing a type erasure
    return underlying_run(input);
};

el_err_code_t PFLD::preprocess() {
    el_err_code_t ret{EL_OK};
    auto*         i_img{static_cast<ImageType*>(this->__p_input)};

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

el_err_code_t PFLD::postprocess() {
    _results.clear();

    // get output
    auto*   data{static_cast<int8_t*>(this->__p_engine->get_output(0))};
    auto    width{_input_img.width};
    auto    height{_input_img.height};
    float   scale{this->__output_quant.scale};
    int32_t zero_point{this->__output_quant.zero_point};
    auto    pred_l{this->__output_shape.dims[1]};

    for (decltype(pred_l) i{0}; i < pred_l; i += 2) {
        _results.emplace_front(
          PointType{.x      = static_cast<decltype(PointType::x)>((data[i] - zero_point) * scale * width),
                    .y      = static_cast<decltype(PointType::y)>((data[i + 1] - zero_point) * scale * height),
                    .target = static_cast<decltype(PointType::target)>(i / 2)});
    }

    return EL_OK;
}

const std::forward_list<PFLD::PointType>& PFLD::get_results() const { return _results; }

}  // namespace edgelab::algorithm

#endif
