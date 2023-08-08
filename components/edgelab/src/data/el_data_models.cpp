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

#include "el_data_models.hpp"

namespace edgelab::data {

Models::Models()
    : __partition_start_addr(0u),
      __partition_size(0u),
      __flash_2_memory_map(nullptr),
      __mmap_handler(),
      __model_id_mask(0u),
      __model_info() {}

Models::~Models() { deinit(); }

el_err_code_t Models::init(const char* partition_name) {
    el_err_code_t ret = el_model_partition_mmap_init(
      partition_name, &__partition_start_addr, &__partition_size, &__flash_2_memory_map, &__mmap_handler);
    if (ret != EL_OK) return ret;
    seek_models_from_flash();
    return ret;
}

void Models::deinit() {
    el_model_partition_mmap_deinit(&__mmap_handler);
    __flash_2_memory_map = nullptr;
    __mmap_handler       = el_model_mmap_handler_t{};
    __model_info.clear();
}

size_t Models::seek_models_from_flash() {
    if (!__flash_2_memory_map) [[unlikely]]
        return 0u;

    __model_id_mask = 0u;
    __model_info.clear();

    static size_t            header_size = sizeof(el_model_header_t);
    const uint8_t*           mem_addr    = nullptr;
    const el_model_header_t* header      = nullptr;
    for (size_t it = 0u; it < __partition_size; it += header_size) {
        mem_addr = __flash_2_memory_map + it;
        header   = reinterpret_cast<const el_model_header_t*>(mem_addr);
        if (!verify_header_magic(header)) continue;

        uint8_t  model_id   = parse_model_id(header);
        uint8_t  model_type = parse_model_type(header);
        uint32_t model_size = parse_model_size(header);
        if (!model_id || !model_type || !model_size || model_size > (__partition_size - it)) [[unlikely]]
            continue;

        if (~__model_id_mask & (1u << model_id)) {
            __model_info.emplace_after(el_model_info_t{.id          = model_id,
                                                       .type        = static_cast<el_algorithm_type_t>(model_type),
                                                       .addr_flash  = __partition_start_addr + it,
                                                       .size        = model_size,
                                                       .addr_memory = mem_addr + header_size});
            __model_id_mask |= (1u << model_id);
        }
        it += model_size;
    }

    return __model_info.size();
}

bool Models::has_model(el_model_id_t model_id) { return __model_id_mask & (1u << model_id); }

el_err_code_t Models::get(el_model_id_t model_id, el_model_info_t& model_info) {
    auto it{std::find_if(
      __model_info.begin(), __model_info.end(), [&](const auto& v) { return __model_info.id == model_id; })};
    if (it != __model_info.end()) [[likely]] {
        model_info = *it;
        return EL_OK;
    }
    return EL_EINVAL;
}

el_model_info_t Models::get_model_info(el_model_id_t model_id) {
    auto it{std::find_if(
      __model_info.begin(), __model_info.end(), [&](const auto& v) { return __model_info.id == model_id; })};
    if (it != __model_info.end()) [[likely]] {
        return *it;
    }
    return el_model_info_t{};
}

std::forward_list<el_model_info_t> Models::get_all_model_info() { return __model_info; }

inline bool Models::verify_header_magic(const el_model_header_t* header) {
    return (el_ntohl(header->b4[0]) & 0xFFFFFF00) == (CONFIG_EL_MODEL_HEADER_MAGIC << 8);
}

inline uint8_t Models::parse_model_id(const el_model_header_t* header) { return header->b1[3] >> 4; }

inline uint8_t Models::parse_model_type(const el_model_header_t* header) { return header->b1[3] & 0x0F; }

inline uint32_t Models::parse_model_size(const el_model_header_t* header) {
    return ((el_ntohl(header->b4[1]) & 0xFFFFFF00) >> 8);
}

}  // namespace edgelab::data
