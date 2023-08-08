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

#include "el_algorithm_fomo.hpp"
#include "el_algorithm_pfld.hpp"
#include "el_algorithm_yolo.hpp"
#include "el_inference_base.h"
#include "el_types.h"

namespace edgelab {

using FOMO = typename edgelab::algorithm::FOMO<edgelab::inference::base::Engine, el_img_t, el_box_t>;
using PFLD = typename edgelab::algorithm::PFLD<edgelab::inference::base::Engine, el_img_t, el_point_t>;
using YOLO = typename edgelab::algorithm::YOLO<edgelab::inference::base::Engine, el_img_t, el_box_t>;

static void register_algorithms() noexcept {
    using namespace edgelab::algorithm::types;
    using namespace edgelab::algorithm::data;

    el_registered_algorithms.emplace(
      0u, el_algorithm_t{.id = 0, .type = 0, .categroy = 0, .input_type = 0, .parameters = {50, 45, 0, 0}});  // YOLO
    el_registered_algorithms.emplace(
      2u, el_algorithm_t{.id = 2, .type = 2, .categroy = 0, .input_type = 0, .parameters = {80, 0, 0, 0}});   // PFLD
    el_registered_algorithms.emplace(
      1u, el_algorithm_t{.id = 1, .type = 1, .categroy = 0, .input_type = 0, .parameters = {0, 0, 0, 0}});    // FOMO
}

}  // namespace edgelab

// TODO: avoid expose this name space globally
using namespace edgelab::algorithm::types;
using namespace edgelab::algorithm::data;
using namespace edgelab::algorithm::utility;

#endif
