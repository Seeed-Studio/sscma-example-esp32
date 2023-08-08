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

#ifndef _EL_ALGORITHM_YOLO_H_
#define _EL_ALGORITHM_YOLO_H_

#include <cstdint>
#include <utility>

#include "el_algorithm_base.hpp"
#include "el_cv.h"
#include "el_debug.h"
#include "el_nms.h"
#include "el_types.h"

namespace edgelab {
namespace algorithm {

template <typename InferenceEngine, typename ImageType, typename BoxType>
class YOLO : public edgelab::algorithm::base::Algorithm<InferenceEngine, ImageType, BoxType> {
    using ScoreType        = edgelab::algorithm::base::Algorithm<InferenceEngine, ImageType, BoxType>::ScoreType;
    using NMSThresholdType = ScoreType;

   private:
    ImageType        _input_img;
    float            _w_scale;
    float            _h_scale;
    NMSThresholdType _nms_threshold;

    enum {
        INDEX_X = 0,
        INDEX_Y = 1,
        INDEX_W = 2,
        INDEX_H = 3,
        INDEX_S = 4,
        INDEX_T = 5,
    };

   protected:
    el_err_code_t preprocess() override;
    el_err_code_t postprocess() override;

   public:
    YOLO(InferenceEngine* engine, ScoreType score_threshold = 50, NMSThresholdType nms_threshold = 45);
    ~YOLO();

    el_err_code_t           set_nms_threshold(ScoreType threshold);
    NMSThresholdType get_nms_threshold() const;
};

template <typename InferenceEngine, typename ImageType, typename BoxType>
YOLO<InferenceEngine, ImageType, BoxType>::YOLO(InferenceEngine* engine,
                                                ScoreType        score_threshold,
                                                NMSThresholdType nms_threshold)
    : edgelab::algorithm::base::Algorithm<InferenceEngine, ImageType, BoxType>(engine, score_threshold),
      _w_scale(1.f),
      _h_scale(1.f),
      _nms_threshold(nms_threshold) {
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
    //   0u, el_algorithm_t{.id = 0, .type = 0, .categroy = 0, .input_type = 0, .parameters = {50, 45, 0, 0}});  // YOLO
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
YOLO<InferenceEngine, ImageType, BoxType>::~YOLO() {
    this->__results.clear();
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
el_err_code_t YOLO<InferenceEngine, ImageType, BoxType>::preprocess() {
    el_err_code_t ret{EL_OK};
    auto*  i_img{this->__p_input};

    // convert image
    ret = rgb_to_rgb(i_img, &_input_img);

    if (ret != EL_OK) {
        return ret;
    }

    for (decltype(ImageType::size) i{0}; i < _input_img.size; ++i) {
        _input_img.data[i] -= 128;
    }

    _w_scale = static_cast<float>(i_img->width) / static_cast<float>(_input_img.width);
    _h_scale = static_cast<float>(i_img->height) / static_cast<float>(_input_img.height);

    return EL_OK;
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
el_err_code_t YOLO<InferenceEngine, ImageType, BoxType>::postprocess() {
    this->__results.clear();

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
        if (score > this->__score_threshold) {
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

            this->__results.emplace_front(std::move(box));
        }
    }
    el_nms(this->__results, _nms_threshold, this->__score_threshold, false, true);

    this->__results.sort([](const BoxType& a, const BoxType& b) { return a.x < b.x; });

    return EL_OK;
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
el_err_code_t YOLO<InferenceEngine, ImageType, BoxType>::set_nms_threshold(NMSThresholdType threshold) {
    _nms_threshold = threshold;
    return EL_OK;
}

template <typename InferenceEngine, typename ImageType, typename BoxType>
YOLO<InferenceEngine, ImageType, BoxType>::NMSThresholdType
  YOLO<InferenceEngine, ImageType, BoxType>::get_nms_threshold() const {
    return _nms_threshold;
}

}  // namespace algorithm
}  // namespace edgelab

#endif
