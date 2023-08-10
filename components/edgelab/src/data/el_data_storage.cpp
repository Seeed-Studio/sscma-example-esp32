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

#include "el_data_storage.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstddef>

#include "el_config_internal.h"
#include "el_debug.h"

#ifdef CONFIG_EL_LIB_FLASHDB

    #include <flashdb.h>

namespace edgelab::data {

Storage::Storage() noexcept : __lock(xSemaphoreCreateCounting(1, 1)), __kvdb(new fdb_kvdb{}) {
    EL_ASSERT(__lock);
    EL_ASSERT(__kvdb);
}

Storage::~Storage() { deinit(); };

el_err_code_t Storage::init(const char* name, const char* path) {
    volatile const Guard guard(this);
    return fdb_kvdb_init(__kvdb, name, path, nullptr, nullptr) == FDB_NO_ERR ? EL_OK : EL_EINVAL;
}

void Storage::deinit() {
    volatile const Guard guard(this);
    if (__kvdb && (fdb_kvdb_deinit(__kvdb) == FDB_NO_ERR)) [[likely]] {
        delete __kvdb;
        __kvdb = nullptr;
    }
}

bool Storage::contains(const char* key) const {
    volatile const Guard guard(this);
    if (!key || !__kvdb) [[unlikely]]
        return false;
    fdb_kv kv{};
    return find_kv(__kvdb, key, &kv);
}

size_t Storage::get_value_size(const char* key) const {
    volatile const Guard guard(this);
    fdb_kv               handler{};
    fdb_kv_t             p_handler = fdb_kv_get_obj(__kvdb, key, &handler);
    if (!p_handler || !p_handler->value_len) [[unlikely]]
        return 0u;

    return p_handler->value_len;
}

bool Storage::erase(const char* key) {
    volatile const Guard guard(this);
    return fdb_kv_del(__kvdb, key) == FDB_NO_ERR;
}

void Storage::clear() {
    volatile const Guard guard(this);
    if (!__kvdb) [[unlikely]]
        return;
    struct fdb_kv_iterator iterator;
    fdb_kv_iterator_init(__kvdb, &iterator);
    while (fdb_kv_iterate(__kvdb, &iterator)) fdb_kv_del(__kvdb, iterator.curr_kv.name);
}

bool Storage::reset() {
    volatile const Guard guard(this);
    return __kvdb ? fdb_kv_set_default(__kvdb) == FDB_NO_ERR : false;
}

}  // namespace edgelab::data

#endif
