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

#ifndef _EL_INFERENCE_TFLITE_H_
#define _EL_INFERENCE_TFLITE_H_

#include "el_inference.h"

#ifdef CONFIG_EL_TFLITE

#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/compatibility.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* opresover */
#ifndef CONFIG_EL_TFLITE_OP_RESOLVER_SIZE
static_assert(false, "CONFIG_EL_TFLITE_OP_RESOLVER_SIZE is not defined");
#else

namespace tflite {

class OpsResolver : public MicroMutableOpResolver<CONFIG_EL_TFLITE_OP_RESOLVER_SIZE> {
   public:
    OpsResolver();

   private:
    TF_LITE_REMOVE_VIRTUAL_DELETE
};

} // namespace tflite

#endif

namespace edgelab {

class TFLiteEngine : public InferenceEngine {
   private:
    static tflite::OpsResolver resolver;
    tflite::MicroInterpreter* interpreter;
    const tflite::Model* model;
    el_memory_pool_t memory_pool;
#ifdef CONFIG_EL_FILESYSTEM
    char* model_file;
#endif

   public:
    TFLiteEngine();
    ~TFLiteEngine();

    EL_ERR init() override;            // Initialize Inference Engine without memory pool
    EL_ERR init(size_t size) override; // Initialize Inference Engine with memory pool
    EL_ERR init(void* pool,
                size_t size) override; // Initialize Inference Engine with memory pool

    EL_ERR run() override; // Run inference

#ifdef CONFIG_EL_FILESYSTEM
    EL_ERR load_model(const char* model_path) override; // Load model from file
#endif
    EL_ERR load_model(const void* model_data,
                      size_t model_size) override; // Load model from memory
    EL_ERR set_input(size_t index,
                     const void* input_data,
                     size_t input_size) override;                   // Set input data
    void* get_input(size_t index) override;                         // Get input data
    void* get_output(size_t index) override;                        // Get output data
    el_shape_t get_input_shape(size_t index) override;              // Get input shape
    el_shape_t get_output_shape(size_t index) override;             // Get output shape
    el_quant_param_t get_input_quant_param(size_t index) override;  // Get input quant param
    el_quant_param_t get_output_quant_param(size_t index) override; // Get output quant param
#ifdef CONFIG_EL_INFERENCER_TENSOR_NAME
    size_t get_input_index(const char* input_name) override;   // Get input index
    size_t get_output_index(const char* output_name) override; // Get output index
    EL_ERR set_input(const char* input_name,
                     const void* input_data,
                     size_t input_size) override;                  // Set input data
    void* get_input(const char* input_name) override;              // Get input data
    void* get_output(const char* output_name) override;            // Get output data
    el_shape_t get_input_shape(const char* input_name) override;   // Get input shape
    el_shape_t get_output_shape(const char* output_name) override; // Get output shape
    el_quant_param_t get_input_quant_param(
        const char* input_name) override; // Get input quant param
    el_quant_param_t get_output_quant_param(
        const char* output_name) override; // Get output quant param
#endif
};

} // namespace edgelab

#endif /* _EL_INFERENCE_TFLITE_H_ */

#endif
