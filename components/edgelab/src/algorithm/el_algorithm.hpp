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

#include <forward_list>

#include "el_algorithm_fomo.hpp"
#include "el_algorithm_imcls.hpp"
#include "el_algorithm_pfld.hpp"
#include "el_algorithm_yolo.hpp"
#include "el_inference.hpp"

namespace edgelab {

namespace algorithm::utility {

el_algorithm_type_t el_algorithm_type_from_engine(const edgelab::InferenceEngine* engine);

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
    using InfoType = typename algorithm::types::el_algorithm_info_t;

    ~AlgorithmDelegate() = default;

    static AlgorithmDelegate* get_delegate();

    InfoType get_algorithm_info(el_algorithm_type_t type) const;

    const std::forward_list<const InfoType*>& get_all_algorithm_info() const;

    size_t get_all_algorithm_info_count() const;

    bool has_algorithm(el_algorithm_type_t type) const;

   protected:
    AlgorithmDelegate();

   private:
    std::forward_list<const InfoType*> _registered_algorithms;
};

}  // namespace edgelab

// TODO: avoid expose this name space globally
using namespace edgelab::algorithm::types;
using namespace edgelab::algorithm::utility;

#endif
