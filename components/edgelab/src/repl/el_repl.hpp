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

#ifndef _EL_REPL_HPP_
#define _EL_REPL_HPP_

#include <algorithm>
#include <deque>
#include <forward_list>
#include <functional>
#include <iostream>
#include <string>
// #include <vector>

#include "el_common.h"
#include "el_compiler.h"
#include "el_debug.h"
#include "el_types.h"

#define EL_CONFIG_AT_CMD_ARGC_MAX (8)

namespace edgelab {

typedef std::function<el_err_code_t(int, char**)> el_repl_cmd_cb_t;

// typedef std::function<el_err_code_t(std::vector<std::string>)> el_repl_cmd_cb_t;

struct el_repl_cmd_t {
    el_repl_cmd_t(std::string cmd, std::string desc, std::string args, el_repl_cmd_cb_t cmd_cb)
        : _cmd(cmd), _desc(desc), _args(args), _argc(0), _cmd_cb(cmd_cb) {
        if (args.size()) _argc = std::count(_args.begin(), _args.end(), ',') + 1;
    }

    ~el_repl_cmd_t() = default;

    friend class ReplServer;

   private:
    std::string      _cmd;
    std::string      _desc;
    std::string      _args;
    uint8_t          _argc;
    el_repl_cmd_cb_t _cmd_cb;
};

class ReplHistory {
   public:
    ReplHistory(int max_size = 10);
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

class ReplServer {
   public:
    ReplServer();
    ~ReplServer();

    ReplServer(ReplServer const&)            = delete;
    ReplServer& operator=(ReplServer const&) = delete;

    void init();
    void deinit();

    void loop(const std::string& line);
    void loop(const char* line, size_t len);
    void loop(char c);

    el_err_code_t register_cmd(const el_repl_cmd_t& cmd);
    el_err_code_t register_cmd(const char* cmd, const char* desc, const char* arg, el_repl_cmd_cb_t cmd_cb);

    el_err_code_t unregister_cmd(const std::string& cmd);
    el_err_code_t unregister_cmd(const char* cmd);

    size_t register_cmds(const std::forward_list<el_repl_cmd_t>& cmd_list);
    size_t register_cmds(const el_repl_cmd_t* cmd_list, size_t size);

    el_err_code_t print_help();

   private:
    el_err_code_t m_exec_cmd(const std::string& line);

    ReplHistory                      _history;
    std::forward_list<el_repl_cmd_t> _cmd_list;

    bool        _is_ctrl;
    std::string _ctrl_line;
    std::string _line;
    int         _line_index;
};

}  // namespace edgelab

#endif
