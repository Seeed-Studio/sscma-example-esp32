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

#ifndef _EL_ALGORITHM_FOMO_HPP_
#define _EL_ALGORITHM_FOMO_HPP_

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
struct el_algorithm_fomo_config_t {
    static constexpr el_algorithm_info_t info{
      .type = EL_ALGO_TYPE_FOMO, .categroy = EL_ALGO_CAT_DET, .input_from = EL_SENSOR_TYPE_CAM};
    uint8_t score_threshold;
};

}  // namespace types

class FOMO : public edgelab::algorithm::base::Algorithm {
    using ImageType  = el_img_t;
    using BoxType    = el_box_t;
    using ConfigType = types::el_algorithm_fomo_config_t;
    using ScoreType  = decltype(types::el_algorithm_fomo_config_t::score_threshold);

   public:
    static constexpr InfoType algorithm_info{types::el_algorithm_fomo_config_t::info};

    FOMO(EngineType* engine, ScoreType score_threshold = 50);
    FOMO(EngineType* engine, const ConfigType& config);
    ~FOMO();

    static bool is_model_valid(const EngineType* engine);

    el_err_code_t                     run(ImageType* input);
    const std::forward_list<BoxType>& get_results() const;

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
    float     _w_scale;
    float     _h_scale;

    std::atomic<ScoreType> _score_threshold;

    std::forward_list<BoxType> _results;
};

FOMO::FOMO(EngineType* engine, ScoreType score_threshold)
    : edgelab::algorithm::base::Algorithm(engine, FOMO::algorithm_info),
      _w_scale(1.f),
      _h_scale(1.f),
      _score_threshold(score_threshold) {
    init();
}

FOMO::FOMO(EngineType* engine, const ConfigType& config)
    : edgelab::algorithm::base::Algorithm(engine, config.info),
      _w_scale(1.f),
      _h_scale(1.f),
      _score_threshold(config.score_threshold) {
    init();
}

FOMO::~FOMO() { _results.clear(); }

bool FOMO::is_model_valid(const EngineType* engine) {
    const auto& input_shape{engine->get_input_shape(0)};
    if (input_shape.size != 4 ||      // B, W, H, C
        input_shape.dims[0] != 1 ||   // B = 1
        input_shape.dims[1] < 16 ||   // W >= 16
        input_shape.dims[2] < 16 ||   // H >= 16
        (input_shape.dims[3] != 3 &&  // C = RGB or Gray
         input_shape.dims[3] != 1))
        return false;

    const auto& output_shape{engine->get_output_shape(0)};
    if (output_shape.size != 4 ||                      // B, W, H, PC
        output_shape.dims[0] != 1 ||                   // B = 1
        output_shape.dims[1] < 2 ||                    // W >= 2
        output_shape.dims[1] > input_shape.dims[1] ||  // W <= IW
        output_shape.dims[2] < 2 ||                    // H >= 2
        output_shape.dims[2] > input_shape.dims[2] ||  // H <= IH
        output_shape.dims[3] < 2                       // PC >= 2
    )
        return false;

    return true;
}

inline void FOMO::init() {
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

el_err_code_t FOMO::run(ImageType* input) {
    _w_scale = static_cast<float>(input->width) / static_cast<float>(_input_img.width);
    _h_scale = static_cast<float>(input->height) / static_cast<float>(_input_img.height);

    // TODO: image type conversion before underlying_run, because underlying_run doing a type erasure
    return underlying_run(input);
};

el_err_code_t FOMO::preprocess() {
    auto* i_img{static_cast<ImageType*>(this->__p_input)};

    // convert image
    rgb_to_rgb(i_img, &_input_img);

    auto size{_input_img.size};
    for (decltype(ImageType::size) i{0}; i < size; ++i) {
        _input_img.data[i] -= 128;
    }

    return EL_OK;
}

el_err_code_t FOMO::postprocess() {
    _results.clear();

    // get output
    auto* data{static_cast<int8_t*>(this->__p_engine->get_output(0))};

    auto width{_input_img.width};
    auto height{_input_img.height};

    float scale{this->__output_quant.scale};

    int32_t zero_point{this->__output_quant.zero_point};

    auto pred_w{this->__output_shape.dims[2]};
    auto pred_h{this->__output_shape.dims[1]};
    auto pred_t{this->__output_shape.dims[3]};

    auto bw{static_cast<decltype(BoxType::w)>(width / pred_w)};
    auto bh{static_cast<decltype(BoxType::h)>(height / pred_h)};

    ScoreType score_threshold{get_score_threshold()};

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
            if (max_score > score_threshold && max_target != 0) {
                // only unsigned is supported for fast div by 2 (>> 1)
                static_assert(std::is_unsigned<decltype(bw)>::value && std::is_unsigned<decltype(bh)>::value);
                _results.emplace_front(BoxType{.x = static_cast<decltype(BoxType::x)>((j * bw + (bw >> 1)) * _w_scale),
                                               .y = static_cast<decltype(BoxType::y)>((i * bh + (bh >> 1)) * _h_scale),
                                               .w = static_cast<decltype(BoxType::w)>(bw * _w_scale),
                                               .h = static_cast<decltype(BoxType::h)>(bh * _h_scale),
                                               .score  = max_score,
                                               .target = max_target});
            }
        }
    }
    _results.sort([](const BoxType& a, const BoxType& b) { return a.x < b.x; });

    return EL_OK;
}

const std::forward_list<FOMO::BoxType>& FOMO::get_results() const { return _results; }

void FOMO::set_score_threshold(ScoreType threshold) { _score_threshold.store(threshold); }

FOMO::ScoreType FOMO::get_score_threshold() const { return _score_threshold.load(); }

void FOMO::set_algorithm_config(const ConfigType& config) { set_score_threshold(config.score_threshold); }

FOMO::ConfigType FOMO::get_algorithm_config() const { return ConfigType{.score_threshold = get_score_threshold()}; }

}  // namespace edgelab::algorithm

#endif
