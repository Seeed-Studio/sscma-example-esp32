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

#ifndef _EL_INFERENCE_H_
#define _EL_INFERENCE_H_

#include <stdio.h>

#include "el_common.h"
#include "el_compiler.h"
#include "el_debug.h"
#include "el_types.h"

#ifdef CONFIG_EL_FILESYSTEM
#include <fstream>
#include <iostream>
#endif

namespace edgelab {

class InferenceEngine {
   public:
    InferenceEngine(){};
    virtual ~InferenceEngine(){}; // Virtual destructor for polymorphism

    virtual EL_ERR init() = 0;            // Initialize Inference Engine without memory pool
    virtual EL_ERR init(size_t size) = 0; // Initialize Inference Engine with memory pool
    virtual EL_ERR init(void* pool,
                        size_t size) = 0; // Initialize Inference Engine with memory pool

    virtual EL_ERR run() = 0; // Run inference

#ifdef CONFIG_EL_FILESYSTEM
    virtual EL_ERR load_model(const char* model_path) = 0; // Load model from file
#endif
    virtual EL_ERR load_model(const void* model_data,
                              size_t model_size) = 0; // Load model from memory

    virtual EL_ERR set_input(size_t index,
                             const void* input_data,
                             size_t input_size) = 0; // Set input data
    virtual void* get_input(size_t index) = 0;       // Get input data

    virtual void* get_output(size_t index) = 0;                        // Get output data
    virtual el_shape_t get_input_shape(size_t index) = 0;              // Get input shape
    virtual el_shape_t get_output_shape(size_t index) = 0;             // Get output shape
    virtual el_quant_param_t get_input_quant_param(size_t index) = 0;  // Get input quant param
    virtual el_quant_param_t get_output_quant_param(size_t index) = 0; // Get output quant param
#ifdef CONFIG_EL_INFERENCER_TENSOR_NAME
    virtual size_t get_input_index(const char* input_name) = 0;   // Get input index
    virtual size_t get_output_index(const char* output_name) = 0; // Get output index
    virtual void* get_input(const char* input_name) = 0;          // Get input data
    virtual EL_ERR set_input(const char* input_name,
                             const void* input_data,
                             size_t input_size) = 0;                  // Set input data
    virtual void* get_output(const char* output_name) = 0;            // Get output data
    virtual el_shape_t get_input_shape(const char* input_name) = 0;   // Get input shape
    virtual el_shape_t get_output_shape(const char* output_name) = 0; // Get output shape
    virtual el_quant_param_t get_input_quant_param(
        const char* input_name) = 0; // Get input quant param
    virtual el_quant_param_t get_output_quant_param(
        const char* output_name) = 0; // Get output quant param
#endif
};

} // namespace edgelab

#endif // _EL_PORTING_H_