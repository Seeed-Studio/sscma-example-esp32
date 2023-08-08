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

#ifndef _EL_ALGORITHM_BASE_HPP_
#define _EL_ALGORITHM_BASE_HPP_

#include <cstdint>
#include <forward_list>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "el_debug.h"
#include "el_types.h"

namespace edgelab {
namespace algorithm {

namespace types {

struct el_algorithm_t {
    uint8_t id;
    uint8_t type;
    uint8_t categroy;
    uint8_t input_type;
    uint8_t parameters[4];
};

}  // namespace types

namespace data {

// later we can use reflection for that
static std::unordered_map<uint8_t, edgelab::algorithm::types::el_algorithm_t> el_registered_algorithms;

}  // namespace data

namespace base {

template <typename InferenceEngine, typename InputType, typename OutputType> class Algorithm {
   protected:
    using ScoreType = decltype(OutputType::score);

   private:
    uint32_t __preprocess_time;   // ms
    uint32_t __run_time;          // ms
    uint32_t __postprocess_time;  // ms

   protected:
    InferenceEngine*              __p_engine;
    InputType*                    __p_input;
    el_shape_t                    __input_shape;
    el_shape_t                    __output_shape;
    el_quant_param_t              __input_quant;
    el_quant_param_t              __output_quant;
    ScoreType                     __score_threshold;
    std::forward_list<OutputType> __results;

    virtual el_err_code_t preprocess()  = 0;
    virtual el_err_code_t postprocess() = 0;

   public:
    Algorithm(InferenceEngine* engine, ScoreType score_threshold = 40);
    virtual ~Algorithm();

    el_err_code_t run(InputType* input);

    uint32_t get_preprocess_time() const;
    uint32_t get_run_time() const;
    uint32_t get_postprocess_time() const;

    el_err_code_t    set_score_threshold(ScoreType threshold);
    ScoreType get_score_threshold() const;

    const std::forward_list<OutputType>& get_results() const;
};

template <typename InferenceEngine, typename InputType, typename OutputType>
Algorithm<InferenceEngine, InputType, OutputType>::Algorithm(InferenceEngine* engine, ScoreType score_threshold)
    : __preprocess_time(0),
      __run_time(0),
      __postprocess_time(0),
      __p_engine(engine),
      __p_input(nullptr),
      __score_threshold(score_threshold) {
    __input_shape  = engine->get_input_shape(0);
    __output_shape = engine->get_output_shape(0);
    __input_quant  = engine->get_input_quant_param(0);
    __output_quant = engine->get_output_quant_param(0);
}

template <typename InferenceEngine, typename InputType, typename OutputType>
Algorithm<InferenceEngine, InputType, OutputType>::~Algorithm() {
    __preprocess_time  = 0;
    __run_time         = 0;
    __postprocess_time = 0;
    __p_engine         = nullptr;
    __p_input          = nullptr;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
el_err_code_t Algorithm<InferenceEngine, InputType, OutputType>::run(InputType* input) {
    el_err_code_t   ret{EL_OK};
    uint32_t start_time{0};
    uint32_t end_time{0};

    EL_ASSERT(__p_engine != nullptr);

    __p_input = input;

    // preprocess
    start_time        = el_get_time_ms();
    ret               = preprocess();
    end_time          = el_get_time_ms();
    __preprocess_time = end_time - start_time;

    if (ret != EL_OK) {
        return ret;
    }

    // run
    start_time = el_get_time_ms();
    ret        = __p_engine->run();
    end_time   = el_get_time_ms();
    __run_time = end_time - start_time;

    if (ret != EL_OK) {
        return ret;
    }

    // postprocess
    start_time         = el_get_time_ms();
    ret                = postprocess();
    end_time           = el_get_time_ms();
    __postprocess_time = end_time - start_time;

    return ret;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
uint32_t Algorithm<InferenceEngine, InputType, OutputType>::get_preprocess_time() const {
    return __preprocess_time;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
uint32_t Algorithm<InferenceEngine, InputType, OutputType>::get_run_time() const {
    return __run_time;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
uint32_t Algorithm<InferenceEngine, InputType, OutputType>::get_postprocess_time() const {
    return __postprocess_time;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
el_err_code_t Algorithm<InferenceEngine, InputType, OutputType>::set_score_threshold(ScoreType threshold) {
    __score_threshold = threshold;
    return EL_OK;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
Algorithm<InferenceEngine, InputType, OutputType>::ScoreType
  Algorithm<InferenceEngine, InputType, OutputType>::get_score_threshold() const {
    return __score_threshold;
}

template <typename InferenceEngine, typename InputType, typename OutputType>
const std::forward_list<OutputType>& Algorithm<InferenceEngine, InputType, OutputType>::get_results() const {
    return __results;
}

}  // namespace base

namespace utility {

template <typename T, typename std::enable_if<std::is_same<T, el_box_t>::value>::type* = nullptr>
std::string el_results_2_string(const std::forward_list<T>& results) {
    auto os{std::ostringstream(std::ios_base::ate)};
    for (const auto& box : results)
        os << "{\"x\": " << unsigned(box.x) << ", \"y\": " << unsigned(box.y) << ", \"w\": " << unsigned(box.w)
           << ", \"h\": " << unsigned(box.h) << ", \"target\": " << unsigned(box.target)
           << ", \"score\": " << unsigned(box.score) << "}, ";
    return os.str();
}

template <typename T, typename std::enable_if<std::is_same<T, el_point_t>::value>::type* = nullptr>
std::string el_results_2_string(const std::forward_list<T>& results) {
    auto os{std::ostringstream(std::ios_base::ate)};
    for (const auto& box : results)
        os << "{\"x\": " << unsigned(box.x) << ", \"y\": " << unsigned(box.y)
           << ", \"target\": " << unsigned(box.target) << "}, ";
    return os.str();
}

}  // namespace utility

}  // namespace algorithm
}  // namespace edgelab

#endif
