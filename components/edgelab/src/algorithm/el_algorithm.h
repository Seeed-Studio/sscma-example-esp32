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

#ifndef _EL_ALGORITHM_H_
#define _EL_ALGORITHM_H_

#include "el_common.h"
#include "el_inference.h"

namespace edgelab {

class Algorithm {
   private:
    uint32_t preprocess_time;  // ms
    uint32_t run_time;         // ms
    uint32_t postprocess_time; // ms

   protected:
    InferenceEngine *engine;
    uint8_t score_threshold;
    uint8_t nms_threshold;
    void *input;
    virtual EL_ERR preprocess() = 0;
    virtual EL_ERR postprocess() = 0;

   public:
    Algorithm(InferenceEngine &engine);
    virtual ~Algorithm();
    virtual EL_ERR init() = 0;
    virtual EL_ERR deinit() = 0;
    EL_ERR run(void *input);
    uint32_t get_preprocess_time();
    uint32_t get_run_time();
    uint32_t get_postprocess_time();
    uint8_t set_score_threshold(uint8_t threshold);
    uint8_t set_nms_threshold(uint8_t threshold);
    uint8_t get_score_threshold();
    uint8_t get_nms_threshold();
    virtual const void *get_result(size_t index) = 0;
    virtual size_t get_result_size() = 0;
};

} // namespace edgelab

#endif /* _EL_ALGO_H_ */
