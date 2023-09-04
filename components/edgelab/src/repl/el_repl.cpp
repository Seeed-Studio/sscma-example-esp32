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

#include "el_repl.hpp"

#include "el_repl_executor.hpp"
#include "el_repl_history.hpp"
#include "el_repl_server.hpp"

namespace edgelab {

using namespace repl;

ReplDelegate* ReplDelegate::get_delegate() {
    static ReplDelegate repl_delegate = ReplDelegate();
    return &repl_delegate;
}

ReplExecutor* ReplDelegate::get_executor_handler() {
    static ReplExecutor* executor_handler = new ReplExecutor{};
    return executor_handler;
}

ReplServer* ReplDelegate::get_server_handler() {
    static ReplServer* server_handler = new ReplServer{};
    return server_handler;
}

}  // namespace edgelab
