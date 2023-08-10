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

#ifndef _EL_ALGORITHM_H_
#define _EL_ALGORITHM_H_

#ifdef __cplusplus

    #include <forward_list>

    #include "el_algorithm_fomo.hpp"
    #include "el_algorithm_pfld.hpp"
    #include "el_algorithm_yolo.hpp"
    #include "el_inference_base.h"
    #include "el_types.h"

namespace edgelab {

using namespace edgelab::algorithm;

using AlgorithmFOMO = class FOMO;
using AlgorithmPFLD = class PFLD;
using AlgorithmYOLO = class YOLO;

class AlgorithmDelegate {
   public:
    ~AlgorithmDelegate() = default;

    static AlgorithmDelegate* get() {
        static AlgorithmDelegate data_delegate = AlgorithmDelegate();
        return &data_delegate;
    }

    const std::forward_list<types::el_algorithm_info_t>& get_registered_algorithms() const {
        return _registered_algorithms;
    }

    // TODO: add singleton algorithm handler

   private:
    AlgorithmDelegate() {
        static uint8_t i = 0u;

    #ifdef _EL_ALGORITHM_FOMO_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = el_algorithm_type_t::ALGORITHM_FOMO,
          .categroy   = 1u,
          .input_type = 1u,
        });
    #endif

    #ifdef _EL_ALGORITHM_PFLD_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = el_algorithm_type_t::ALGORITHM_PFLD,
          .categroy   = 1u,
          .input_type = 1u,
        });
    #endif

    #ifdef _EL_ALGORITHM_YOLO_HPP_
        _registered_algorithms.emplace_front(types::el_algorithm_info_t{
          .id         = ++i,
          .type       = el_algorithm_type_t::ALGORITHM_YOLO,
          .categroy   = 1u,
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

#endif
