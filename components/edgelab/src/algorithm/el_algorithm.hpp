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

#ifndef _EL_ALGORITHM_HPP_
#define _EL_ALGORITHM_HPP_

#include <algorithm>
#include <forward_list>

#include "el_algorithm_fomo.hpp"
#include "el_algorithm_imcls.hpp"
#include "el_algorithm_pfld.hpp"
#include "el_algorithm_yolo.hpp"
#include "el_inference.hpp"
#include "el_types.h"

namespace edgelab {

namespace algorithm::utility {

el_algorithm_type_t el_algorithm_type_from_engine(const edgelab::InferenceEngine* engine) {
#ifdef _EL_ALGORITHM_FOMO_HPP_
    if (algorithm::FOMO::is_model_valid(engine)) return EL_ALGO_TYPE_FOMO;
#endif
#ifdef _EL_ALGORITHM_PFLD_HPP_
    if (algorithm::PFLD::is_model_valid(engine)) return EL_ALGO_TYPE_PFLD;
#endif
#ifdef _EL_ALGORITHM_YOLO_HPP_
    if (algorithm::YOLO::is_model_valid(engine)) return EL_ALGO_TYPE_YOLO;
#endif
#ifdef _EL_ALGORITHM_IMCLS_HPP_
    if (algorithm::IMCLS::is_model_valid(engine)) return EL_ALGO_TYPE_IMCLS;
#endif
    return EL_ALGO_TYPE_UNDEFINED;
}

template <typename T> constexpr std::string el_results_2_json(const std::forward_list<T>& results) {
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

}  // namespace algorithm::utility

using Algorithm = class algorithm::base::Algorithm;

#ifdef _EL_ALGORITHM_FOMO_HPP_
using AlgorithmFOMO = class algorithm::FOMO;
#endif
#ifdef _EL_ALGORITHM_PFLD_HPP_
using AlgorithmPFLD = class algorithm::PFLD;
#endif
#ifdef _EL_ALGORITHM_YOLO_HPP_
using AlgorithmYOLO = class algorithm::YOLO;
#endif
#ifdef _EL_ALGORITHM_IMCLS_HPP_
using AlgorithmIMCLS = class algorithm::IMCLS;
#endif

class AlgorithmDelegate {
   public:
    ~AlgorithmDelegate() = default;

    static AlgorithmDelegate* get_delegate() {
        static AlgorithmDelegate data_delegate = AlgorithmDelegate();
        return &data_delegate;
    }

    const algorithm::types::el_algorithm_info_t& get_algorithm_info(el_algorithm_type_t type) const {
        auto it = std::find_if(_registered_algorithms.begin(),
                               _registered_algorithms.end(),
                               [&](const algorithm::types::el_algorithm_info_t& i) { return i.type == type; });
        if (it != _registered_algorithms.end()) return *it;
        static algorithm::types::el_algorithm_info_t algorithm_info{};
        return algorithm_info;
    }

    const std::forward_list<algorithm::types::el_algorithm_info_t>& get_all_algorithm_info() const {
        return _registered_algorithms;
    }

    size_t get_all_algorithm_info_count() const {
        return std::distance(_registered_algorithms.begin(), _registered_algorithms.end());
    }

    bool has_algorithm(el_algorithm_type_t type) const {
        auto it = std::find_if(_registered_algorithms.begin(),
                               _registered_algorithms.end(),
                               [&](const algorithm::types::el_algorithm_info_t& i) { return i.type == type; });
        return it != _registered_algorithms.end();
    }

    // TODO: add singleton algorithm handler

   private:
    AlgorithmDelegate() {
#ifdef _EL_ALGORITHM_FOMO_HPP_
        _registered_algorithms.emplace_front(AlgorithmFOMO::algorithm_info);
#endif
#ifdef _EL_ALGORITHM_PFLD_HPP_
        _registered_algorithms.emplace_front(AlgorithmPFLD::algorithm_info);
#endif
#ifdef _EL_ALGORITHM_YOLO_HPP_
        _registered_algorithms.emplace_front(AlgorithmYOLO::algorithm_info);
#endif
#ifdef _EL_ALGORITHM_IMCLS_HPP_
        _registered_algorithms.emplace_front(AlgorithmIMCLS::algorithm_info);
#endif
    }

    std::forward_list<algorithm::types::el_algorithm_info_t> _registered_algorithms;
};

}  // namespace edgelab

// TODO: avoid expose this name space globally
using namespace edgelab::algorithm::types;
using namespace edgelab::algorithm::utility;

#endif
