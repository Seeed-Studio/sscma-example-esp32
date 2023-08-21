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

#include "el_algorithm_cls.hpp"
#include "el_algorithm_fomo.hpp"
#include "el_algorithm_pfld.hpp"
#include "el_algorithm_yolo.hpp"
#include "el_inference_base.hpp"
#include "el_types.h"

namespace edgelab {

using namespace edgelab::algorithm;

using Algorithm     = class base::Algorithm;
using AlgorithmCLS  = class CLS;
using AlgorithmFOMO = class FOMO;
using AlgorithmPFLD = class PFLD;
using AlgorithmYOLO = class YOLO;

class AlgorithmDelegate {
   public:
    ~AlgorithmDelegate() = default;

    static AlgorithmDelegate* get_delegate() {
        static AlgorithmDelegate data_delegate = AlgorithmDelegate();
        return &data_delegate;
    }

    const types::el_algorithm_info_t& get_algorithm_info(uint8_t id) const {
        auto it = std::find_if(_registered_algorithms.begin(),
                               _registered_algorithms.end(),
                               [&](const types::el_algorithm_info_t& i) { return i.id == id; });
        if (it != _registered_algorithms.end()) return *it;
        static types::el_algorithm_info_t algorithm_info{};
        return algorithm_info;
    }

    const std::forward_list<types::el_algorithm_info_t>& get_all_algorithm_info() const {
        return _registered_algorithms;
    }

    size_t get_all_algorithm_info_count() const {
        return std::distance(_registered_algorithms.begin(), _registered_algorithms.end());
    }

    bool has_algorithm(uint8_t id) const {
        auto it = std::find_if(_registered_algorithms.begin(),
                               _registered_algorithms.end(),
                               [&](const types::el_algorithm_info_t& i) { return i.id == id; });
        return it != _registered_algorithms.end();
    }

    // TODO: add singleton algorithm handler

   private:
    AlgorithmDelegate() {
        static uint8_t i = 0u;

#ifdef _EL_ALGORITHM_FOMO_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = EL_ALGO_TYPE_FOMO,
          .categroy   = 1u,
          .input_type = 1u,
        });
#endif

#ifdef _EL_ALGORITHM_PFLD_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = EL_ALGO_TYPE_PFLD,
          .categroy   = 1u,
          .input_type = 1u,
        });
#endif

#ifdef _EL_ALGORITHM_YOLO_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = EL_ALGO_TYPE_YOLO,
          .categroy   = 1u,
          .input_type = 1u,
        });
#endif

#ifdef _EL_ALGORITHM_CLS_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = EL_ALGO_TYPE_CLS,
          .categroy   = 2u,
          .input_type = 1u,
        });
#endif
    }

    std::forward_list<types::el_algorithm_info_t> _registered_algorithms;
};

}  // namespace edgelab

// TODO: avoid expose this name space globally
using namespace edgelab::algorithm::types;
using namespace edgelab::algorithm::utility;

#endif
