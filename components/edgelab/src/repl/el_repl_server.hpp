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

#ifndef _EL_REPL_SERVER_HPP_
#define _EL_REPL_SERVER_HPP_

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <forward_list>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "el_compiler.h"
#include "el_debug.h"
#include "el_repl_history.hpp"
#include "el_types.h"

namespace edgelab::repl {

class ReplServer;

namespace types {

typedef std::function<void(const std::string&)> el_repl_echo_cb_t;

typedef std::function<el_err_code_t(std::vector<std::string>)> el_repl_cmd_cb_t;

struct el_repl_cmd_t {
    el_repl_cmd_t(std::string cmd_, std::string desc_, std::string args_, el_repl_cmd_cb_t cmd_cb_)
        : cmd(cmd_), desc(desc_), args(args_), cmd_cb(cmd_cb_), _argc(0) {
        if (args.size()) _argc = std::count(args.begin(), args.end(), ',') + 1;
    }

    ~el_repl_cmd_t() = default;

    std::string      cmd;
    std::string      desc;
    std::string      args;
    el_repl_cmd_cb_t cmd_cb;

    friend class edgelab::repl::ReplServer;

   private:
    uint8_t _argc;
};

}  // namespace types

class ReplServer {
   public:
    ReplServer();
    ~ReplServer();

    ReplServer(ReplServer const&)            = delete;
    ReplServer& operator=(ReplServer const&) = delete;

    void init(types::el_repl_echo_cb_t echo_cb = [](const std::string& str) { el_printf(str.c_str()); });
    void deinit();

    bool has_cmd(const std::string& cmd);

    el_err_code_t register_cmd(const types::el_repl_cmd_t& cmd);
    el_err_code_t register_cmd(const char* cmd, const char* desc, const char* arg, types::el_repl_cmd_cb_t cmd_cb);

    template <typename... Args> void unregister_cmd(Args&&... args) {
        const Guard guard(this);
        ((m_unregister_cmd(std::forward<Args>(args))), ...);
    }

    std::forward_list<types::el_repl_cmd_t> get_registered_cmds() const;
    void                                    print_help();

    void loop(const std::string& line);
    void loop(char c);

   protected:
    void m_unregister_cmd(const std::string& cmd);

    el_err_code_t m_exec_cmd(const std::string& line);

    template <typename... Args> inline void m_echo_cb(Args&&... args) {
        auto os{std::ostringstream(std::ios_base::ate)};
        ((os << (std::forward<Args>(args))), ...);
        _echo_cb(os.str());
    }

    inline void m_lock() const { xSemaphoreTake(_cmd_list_lock, portMAX_DELAY); }
    inline void m_unlock() const { xSemaphoreGive(_cmd_list_lock); }

    struct Guard {
        Guard(const ReplServer* const repl_server) noexcept : __repl_server(repl_server) { __repl_server->m_lock(); }
        ~Guard() noexcept { __repl_server->m_unlock(); }

        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;

       private:
        const ReplServer* const __repl_server;
    };

   private:
    ReplHistory _history;

    mutable SemaphoreHandle_t               _cmd_list_lock;
    std::forward_list<types::el_repl_cmd_t> _cmd_list;

    types::el_repl_echo_cb_t _echo_cb;

    bool        _is_ctrl;
    std::string _ctrl_line;
    std::string _line;
    int         _line_index;
};

}  // namespace edgelab::repl

#endif
