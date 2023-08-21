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

#include "el_repl_history.hpp"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <string>

#include "el_common.h"
#include "el_compiler.h"
#include "el_debug.h"
#include "el_types.h"

namespace edgelab::repl {

ReplHistory::ReplHistory(int max_size) : _history_index(-1), _max_size(max_size) {}

el_err_code_t ReplHistory::add(const std::string& line) {
    if (line.empty()) [[unlikely]]
        return EL_OK;

    if (!_history.empty() && _history.back().compare(line) == 0) return EL_OK;

    auto it =
      std::find_if(_history.begin(), _history.end(), [&](const std::string& l) { return l.compare(line) == 0; });

    if (it != _history.end()) [[likely]]
        _history.erase(it);

    while (_history.size() >= _max_size) _history.pop_front();

    _history.push_back(line);
    _history.shrink_to_fit();
    _history_index = _history.size() - 1;

    return EL_OK;
}

el_err_code_t ReplHistory::add(const char* line) { return add(std::string(line)); }

el_err_code_t ReplHistory::get(std::string& line, int index) {
    if (index < 0 || index >= _history.size()) [[unlikely]]
        return EL_ELOG;

    line = _history[index];

    return EL_OK;
}

el_err_code_t ReplHistory::next(std::string& line) {
    if (_history.empty()) [[unlikely]]
        return EL_ELOG;

    if (_history_index < 0) [[unlikely]]
        _history_index = _history.size() - 1;

    if (_history_index < (_history.size() - 1)) [[likely]]
        ++_history_index;

    line = _history[_history_index];

    return EL_OK;
}

el_err_code_t ReplHistory::prev(std::string& line) {
    if (_history.empty()) [[unlikely]]
        return EL_ELOG;

    if (_history_index >= 0) [[likely]]
        line = _history[_history_index--];

    return EL_OK;
}

bool ReplHistory::reset() {
    bool is_tail   = _history_index == (_history.size() - 1);
    _history_index = _history.size() - 1;

    return is_tail;
}

el_err_code_t ReplHistory::clear() {
    _history.clear();
    _history.shrink_to_fit();
    _history_index = -1;

    return EL_OK;
}

size_t ReplHistory::size() const { return _history.size(); };

void ReplHistory::print() const {
    for (const auto& line : _history) el_printf("%s\n", line.c_str());
}

}  // namespace edgelab::repl
