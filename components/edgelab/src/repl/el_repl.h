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

#ifndef _EL_REPL_H_
#define _EL_REPL_H_

#include <deque>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "el_common.h"
#include "el_compiler.h"
#include "el_debug.h"
#include "el_types.h"

namespace edgelab {

typedef std::function<el_err_code_t(void)>        el_repl_cmd_exec_cb_t;
typedef std::function<el_err_code_t(void)>        el_repl_cmd_read_cb_t;
typedef std::function<el_err_code_t(int, char**)> el_repl_cmd_write_cb_t;

typedef enum {
    EL_REPL_CMD_NONE  = 0x00,
    EL_REPL_CMD_EXEC  = EL_BIT(0),
    EL_REPL_CMD_READ  = EL_BIT(1),
    EL_REPL_CMD_WRITE = EL_BIT(2),
} el_repl_cmd_type_t;

typedef struct {
    std::string            cmd;
    std::string            desc;
    std::string            arg;
    el_repl_cmd_exec_cb_t  exec_cb;
    el_repl_cmd_read_cb_t  read_cb;
    el_repl_cmd_write_cb_t write_cb;
} el_repl_cmd_t;

class ReplHistory {
   private:
    std::deque<std::string> _history;
    int                     _history_index;
    int                     _max_size;

   public:
    ReplHistory(int max_size = 10) : _max_size(max_size) { _history_index = -1; };
    ~ReplHistory(){};

    el_err_code_t add(std::string& line);
    el_err_code_t add(const char* line) {
        std::string str(line);
        return add(str);
    }

    el_err_code_t get(std::string& line, int index);
    el_err_code_t next(std::string& line);
    el_err_code_t prev(std::string& line);
    el_err_code_t clear();
    int    size() { return _history.size(); };
    void   print();
};

class ReplServer {
   private:
    std::vector<el_repl_cmd_t> _cmd_list;
    ReplHistory                _history;
    bool                       _is_ctrl;
    std::string                _line;
    int                        _line_index;
    std::string                _ctrl_line;
    static ReplServer*         _instance;

   public:
    ReplServer() { _line_index = -1; };
    ~ReplServer(){};

    static ReplServer* get_instance() {
        if (!_instance) _instance = new ReplServer();
        return _instance;
    }

    ReplServer(ReplServer const&)            = delete;
    ReplServer& operator=(ReplServer const&) = delete;

    void init();
    void loop(std::string& line);
    void loop(const char* line, size_t len);
    void loop(char c);

    el_err_code_t register_cmd(el_repl_cmd_t& cmd);
    el_err_code_t register_cmd(const char*            cmd,
                        const char*            desc,
                        const char*            arg,
                        el_repl_cmd_exec_cb_t  exec_cb,
                        el_repl_cmd_read_cb_t  read_cb,
                        el_repl_cmd_write_cb_t write_cb);
    el_err_code_t register_cmds(std::vector<el_repl_cmd_t>& cmd_list);
    el_err_code_t register_cmds(el_repl_cmd_t* cmd_list, int size);
    el_err_code_t unregister_cmd(std::string& cmd);
    el_err_code_t print_help();

   private:
    el_err_code_t _exec_cmd(std::string& line);
};

}  // namespace edgelab

#endif /* _EL_REPL_H_ */