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

#ifndef _EL_DATA_HPP_
#define _EL_DATA_HPP_

#include "el_data_models.hpp"
#include "el_data_storage.hpp"

namespace edgelab {

using Models = data::Models;

#ifdef CONFIG_EL_LIB_FLASHDB
using Storage = data::Storage;
#endif

class DataDelegate {
   public:
    ~DataDelegate() = default;

    static DataDelegate* get_delegate();

    Models* get_models_handler();

#ifdef CONFIG_EL_LIB_FLASHDB
    Storage* get_storage_handler();
#endif

   protected:
    DataDelegate() = default;
};

}  // namespace edgelab

// TODO: avoid expose this name space globally
using namespace edgelab::data::types;
using namespace edgelab::data::utility;

#endif
