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

#ifndef _EL_REPL_HISTORY_HPP_
#define _EL_REPL_HISTORY_HPP_

#include <cstdint>
#include <deque>
#include <string>

#include "el_compiler.h"
#include "el_debug.h"
#include "el_types.h"

#define CONFIG_EL_REPL_HISTORY_MAX (8)

namespace edgelab::repl {

class ReplHistory {
   public:
    ReplHistory(int max_size = CONFIG_EL_REPL_HISTORY_MAX);
    ~ReplHistory() = default;

    el_err_code_t add(const std::string& line);
    el_err_code_t add(const char* line);

    el_err_code_t get(std::string& line, int index);
    el_err_code_t next(std::string& line);
    el_err_code_t prev(std::string& line);
    bool          reset();

    el_err_code_t clear();
    size_t        size() const;

    void print() const;

   private:
    std::deque<std::string> _history;
    int                     _history_index;
    int                     _max_size;
};

}  // namespace edgelab::repl

#endif
