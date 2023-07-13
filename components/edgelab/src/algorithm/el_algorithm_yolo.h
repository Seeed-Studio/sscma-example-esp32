/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Hongtai Liu (Seeed Technology Inc.)
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

#ifndef _EL_ALGO_YOLO_H_
#define _EL_ALGO_YOLO_H_

#include <forward_list>

#include "el_algorithm.h"
#include "el_types.h"

namespace edgelab {

class Yolo : public Algorithm {
   private:
    std::forward_list<el_box_t> results;
    size_t result_size;
    el_shape_t input_shape;
    el_shape_t output_shape;
    el_quant_param_t input_quant;
    el_quant_param_t output_quant;
    float w_scale, h_scale;

   protected:
    EL_ERR preprocess() override;
    EL_ERR postprocess() override;

   public:
    Yolo(InferenceEngine &engine);
    ~Yolo();
    EL_ERR init() override;
    EL_ERR deinit() override;
    const void *get_result(size_t index) override;
    size_t get_result_size() override;
    enum {
        INDEX_X = 0,
        INDEX_Y = 1,
        INDEX_W = 2,
        INDEX_H = 3,
        INDEX_S = 4,
        INDEX_T = 5,
    };
};

} // namespace edgelab

#endif /* _EL_ALGO_YOLO_H_ */