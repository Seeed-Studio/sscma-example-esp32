/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 nullptr (Seeed Technology Inc.)
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

#ifndef _EL_DATA_MODELS_HPP_
#define _EL_DATA_MODELS_HPP_

#include <algorithm>
#include <cstdint>
#include <forward_list>

#include "el_compiler.h"
#include "el_config_internal.h"
#include "el_debug.h"
#include "el_flash.h"
#include "el_types.h"

namespace edgelab::data {

class Models {
   public:
    Models();
    ~Models();

    Models(const Models&)            = delete;
    Models& operator=(const Models&) = delete;

    el_err_code_t init(const char* partition_name = CONFIG_EL_MODEL_PARTITION_NAME);
    void          deinit();

    size_t                             seek_models_from_flash();
    bool                               has_model(el_model_id_t model_id);
    el_err_code_t                      get(el_model_id_t model_id, el_model_info_t& model_info);
    el_model_info_t                    get_model_info(el_model_id_t model_id);
    std::forward_list<el_model_info_t> get_all_model_info();

   protected:
    bool     verify_header_magic(const el_model_header_t* header);
    uint8_t  parse_model_id(const el_model_header_t* header);
    uint8_t  parse_model_type(const el_model_header_t* header);
    uint32_t parse_model_size(const el_model_header_t* header);

   private:
    uint32_t                           __partition_start_addr;
    uint32_t                           __partition_size;
    const uint8_t*                     __flash_2_memory_map;
    el_model_mmap_handler_t            __mmap_handler;
    uint16_t                           __model_id_mask;
    std::forward_list<el_model_info_t> __model_info;
};

}  // namespace edgelab::data

#endif
