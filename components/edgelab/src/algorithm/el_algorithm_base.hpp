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
#include <functional>
#include <sstream>
#include <string>
#include <type_traits>

#include "el_debug.h"
#include "el_inference_base.h"
#include "el_types.h"

namespace edgelab::algorithm {

namespace types {

struct el_algorithm_info_t {
    uint8_t             id;
    el_algorithm_type_t type;
    uint8_t             categroy;
    uint8_t             input_type;
};

}  // namespace types

namespace base {

class Algorithm {
   public:
    Algorithm(edgelab::inference::base::Engine* engine);
    virtual ~Algorithm();

    uint32_t get_preprocess_time() const;
    uint32_t get_run_time() const;
    uint32_t get_postprocess_time() const;

   protected:
    el_err_code_t underlying_run(void* input);

    virtual el_err_code_t preprocess()  = 0;
    virtual el_err_code_t postprocess() = 0;

    edgelab::inference::base::Engine* __p_engine;
    void*                             __p_input;
    el_shape_t                        __input_shape;
    el_shape_t                        __output_shape;
    el_quant_param_t                  __input_quant;
    el_quant_param_t                  __output_quant;

   private:
    uint32_t __preprocess_time;   // ms
    uint32_t __run_time;          // ms
    uint32_t __postprocess_time;  // ms
};

Algorithm::Algorithm(edgelab::inference::base::Engine* engine)
    : __p_engine(engine), __p_input(nullptr), __preprocess_time(0), __run_time(0), __postprocess_time(0) {
    __input_shape  = engine->get_input_shape(0);
    __output_shape = engine->get_output_shape(0);
    __input_quant  = engine->get_input_quant_param(0);
    __output_quant = engine->get_output_quant_param(0);
}

Algorithm::~Algorithm() {
    __preprocess_time  = 0;
    __run_time         = 0;
    __postprocess_time = 0;
    __p_engine         = nullptr;
    __p_input          = nullptr;
}

el_err_code_t Algorithm::underlying_run(void* input) {
    el_err_code_t ret{EL_OK};
    uint32_t      start_time{0};
    uint32_t      end_time{0};

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

uint32_t Algorithm::get_preprocess_time() const { return __preprocess_time; }

uint32_t Algorithm::get_run_time() const { return __run_time; }

uint32_t Algorithm::get_postprocess_time() const { return __postprocess_time; }

}  // namespace base

namespace utility {

template <typename T> std::string el_results_2_json(const std::forward_list<T>& results) {
    auto os{std::ostringstream(std::ios_base::ate)};
    using F                = std::function<void(void)>;
    static F delim_f       = []() {};
    static F print_delim_f = [&os]() { os << ", "; };
    static F print_void_f  = [&]() { delim_f = print_delim_f; };
    delim_f                = print_void_f;
    os << "[";
    if constexpr (std::is_same<T, el_box_t>::value) {
        for (const auto& box : results) {
            delim_f();
            os << "{\"x\": " << unsigned(box.x) << ", \"y\": " << unsigned(box.y) << ", \"w\": " << unsigned(box.w)
               << ", \"h\": " << unsigned(box.h) << ", \"target\": " << unsigned(box.target)
               << ", \"score\": " << unsigned(box.score) << "}";
        }
    } else if constexpr (std::is_same<T, el_point_t>::value) {
        for (const auto& point : results) {
            delim_f();
            os << "{\"x\": " << unsigned(point.x) << ", \"y\": " << unsigned(point.y)
               << ", \"target\": " << unsigned(point.target) << "}";
        }
    } else if constexpr (std::is_same<T, el_class_t>::value) {
        for (const auto& cls : results) {
            delim_f();
            os << "{\"score\": " << unsigned(cls.score) << ", \"target\": " << unsigned(cls.target) << "}";
        }
    }
    os << "]";
    return os.str();
}

}  // namespace utility

}  // namespace edgelab::algorithm

#endif
