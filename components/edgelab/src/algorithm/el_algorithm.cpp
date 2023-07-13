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

#include "el_algorithm.h"

namespace edgelab {

Algorithm::Algorithm(InferenceEngine &engine)
{
    this->engine = &engine;
    preprocess_time = 0;
    run_time = 0;
    postprocess_time = 0;
    score_threshold = 40;
    nms_threshold = 20;
}

Algorithm::~Algorithm()
{
    this->engine = nullptr;
}

EL_ERR Algorithm::run(void *input)
{
    EL_ERR ret = EL_OK;
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    EL_ASSERT(this->engine != nullptr);

    this->input = input;

    // preprocess
    start_time = el_get_time_ms();
    ret = this->preprocess();
    end_time = el_get_time_ms();
    preprocess_time = end_time - start_time;

    if (ret != EL_OK) {
        return ret;
    }

    // run
    start_time = el_get_time_ms();
    ret = this->engine->run();
    end_time = el_get_time_ms();
    run_time = end_time - start_time;

    if (ret != EL_OK) {
        return ret;
    }

    // postprocess
    start_time = el_get_time_ms();
    ret = this->postprocess();
    end_time = el_get_time_ms();
    postprocess_time = end_time - start_time;

    return ret;
}

uint32_t Algorithm::get_postprocess_time()
{
    return postprocess_time;
}

uint32_t Algorithm::get_preprocess_time()
{
    return preprocess_time;
}

uint32_t Algorithm::get_run_time()
{
    return run_time;
}

uint8_t Algorithm::set_score_threshold(uint8_t threshold)
{
    score_threshold = threshold;
    return score_threshold;
}

uint8_t Algorithm::set_nms_threshold(uint8_t threshold)
{
    nms_threshold = threshold;
    return nms_threshold;
}

uint8_t Algorithm::get_score_threshold()
{
    return score_threshold;
}

uint8_t Algorithm::get_nms_threshold()
{
    return nms_threshold;
}

} // namespace edgelab
