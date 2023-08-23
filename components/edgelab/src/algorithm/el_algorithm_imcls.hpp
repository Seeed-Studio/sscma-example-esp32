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

#ifndef _EL_ALGORITHM_IMCLS_HPP_
#define _EL_ALGORITHM_IMCLS_HPP_

#include <atomic>
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
struct el_algorithm_imcls_config_t {
    static constexpr el_algorithm_info_t info{
      .type = EL_ALGO_TYPE_IMCLS, .categroy = EL_ALGO_CAT_CLS, .input_from = EL_SENSOR_TYPE_CAM};
    uint8_t score_threshold;
};

}  // namespace types

class IMCLS : public edgelab::algorithm::base::Algorithm {
    using ImageType  = el_img_t;
    using ClassType  = el_class_t;
    using ConfigType = types::el_algorithm_imcls_config_t;
    using ScoreType  = decltype(types::el_algorithm_imcls_config_t::score_threshold);

   public:
    static constexpr InfoType algorithm_info{types::el_algorithm_imcls_config_t::info};

    IMCLS(EngineType* engine, ScoreType score_threshold = 50);
    IMCLS(EngineType* engine, const ConfigType& config);
    ~IMCLS();

    static bool is_model_valid(const EngineType* engine);

    el_err_code_t                       run(ImageType* input);
    const std::forward_list<ClassType>& get_results() const;

    void      set_score_threshold(ScoreType threshold);
    ScoreType get_score_threshold() const;

    void       set_algorithm_config(const ConfigType& config);
    ConfigType get_algorithm_config() const;

   protected:
    inline void init();

    el_err_code_t preprocess() override;
    el_err_code_t postprocess() override;

   private:
    ImageType _input_img;

    std::atomic<ScoreType> _score_threshold;

    std::forward_list<ClassType> _results;
};

IMCLS::IMCLS(EngineType* engine, ScoreType score_threshold)
    : edgelab::algorithm::base::Algorithm(engine, IMCLS::algorithm_info), _score_threshold(score_threshold) {
    init();
}

IMCLS::IMCLS(EngineType* engine, const ConfigType& config)
    : edgelab::algorithm::base::Algorithm(engine, config.info), _score_threshold(config.score_threshold) {
    init();
}

IMCLS::~IMCLS() { _results.clear(); }

bool IMCLS::is_model_valid(const EngineType* engine) {
    const auto& input_shape{engine->get_input_shape(0)};
    if (input_shape.size != 4 ||      // B, W, H, C
        input_shape.dims[0] != 1 ||   // B = 1
        input_shape.dims[1] < 16 ||   // W >= 16
        input_shape.dims[2] < 16 ||   // H >= 16
        (input_shape.dims[3] != 3 &&  // C = RGB or Gray
         input_shape.dims[3] != 1))
        return false;

    const auto& output_shape{engine->get_output_shape(0)};
    if (output_shape.size != 2 ||     // B, C
        output_shape.dims[0] != 1 ||  // B = 1
        output_shape.dims[1] < 2      // C >= 2
    )
        return false;

    return true;
}

inline void IMCLS::init() {
    EL_ASSERT(is_model_valid(this->__p_engine));
    EL_ASSERT(_score_threshold.is_lock_free());

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

el_err_code_t IMCLS::run(ImageType* input) {
    // TODO: image type conversion before underlying_run, because underlying_run doing a type erasure
    return underlying_run(input);
};

el_err_code_t IMCLS::preprocess() {
    auto* i_img{static_cast<ImageType*>(this->__p_input)};

    // convert image
    rgb_to_rgb(i_img, &_input_img);

    auto size{_input_img.size};
    for (decltype(ImageType::size) i{0}; i < size; ++i) {
        _input_img.data[i] -= 128;
    }

    return EL_OK;
}

el_err_code_t IMCLS::postprocess() {
    _results.clear();

    // get output
    auto* data{static_cast<int8_t*>(this->__p_engine->get_output(0))};

    float scale{this->__output_quant.scale};
    bool  rescale{scale < 0.1f ? true : false};

    int32_t zero_point{this->__output_quant.zero_point};

    auto pred_l{this->__output_shape.dims[1]};

    ScoreType score_threshold{get_score_threshold()};

    for (decltype(pred_l) i{0}; i < pred_l; ++i) {
        auto score{static_cast<decltype(scale)>(data[i] - zero_point) * scale};
        score = rescale ? score * 100.f : score;
        if (score > score_threshold)
            _results.emplace_front(ClassType{.score  = static_cast<decltype(ClassType::score)>(score),
                                             .target = static_cast<decltype(ClassType::target)>(i)});
    }
    _results.sort([](const ClassType& a, const ClassType& b) { return a.score > b.score; });

    return EL_OK;
}

const std::forward_list<IMCLS::ClassType>& IMCLS::get_results() const { return _results; }

void IMCLS::set_score_threshold(ScoreType threshold) { _score_threshold.store(threshold); }

IMCLS::ScoreType IMCLS::get_score_threshold() const { return _score_threshold.load(); }

void IMCLS::set_algorithm_config(const ConfigType& config) { set_score_threshold(config.score_threshold); }

IMCLS::ConfigType IMCLS::get_algorithm_config() const { return ConfigType{.score_threshold = get_score_threshold()}; }

}  // namespace edgelab::algorithm

#endif
