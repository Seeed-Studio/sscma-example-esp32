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

typedef std::function<void(el_err_code_t, const std::string&)> el_repl_echo_cb_t;

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

    void init(types::el_repl_echo_cb_t echo_cb = [](el_err_code_t, const std::string& str) { el_printf(str.c_str()); });
    void deinit();

    bool has_cmd(const std::string& cmd);

    el_err_code_t register_cmd(const types::el_repl_cmd_t& cmd);
    el_err_code_t register_cmd(const char* cmd, const char* desc, const char* arg, types::el_repl_cmd_cb_t cmd_cb);

    template <typename... Args> void unregister_cmd(Args&&... args) {
        const Guard guard(this, _cmd_list_lock);
        ((m_unregister_cmd(std::forward<Args>(args))), ...);
    }

    std::forward_list<types::el_repl_cmd_t> get_registered_cmds() const;
    void                                    print_help();

    el_err_code_t exec_non_lock(std::string line);
    el_err_code_t exec(std::string line);
    void          loop(const std::string& line);
    void          loop(char c);

   protected:
    struct Guard {
        Guard(const ReplServer* const repl_server, SemaphoreHandle_t& lock) noexcept
            : __repl_server(repl_server), __lock(lock) {
            __repl_server->m_lock(__lock);
        }
        ~Guard() noexcept { __repl_server->m_unlock(__lock); }

        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;

       private:
        const ReplServer* const __repl_server;
        SemaphoreHandle_t&      __lock;
    };

    void m_unregister_cmd(const std::string& cmd);

    el_err_code_t m_exec_cmd(const std::string& line);

    template <typename... Args> inline void m_echo_cb(el_err_code_t ret, Args&&... args) {
        auto os{std::ostringstream(std::ios_base::ate)};
        ((os << (std::forward<Args>(args))), ...);
        _echo_cb(ret, os.str());
    }

    inline void m_lock(SemaphoreHandle_t lock) const { xSemaphoreTake(lock, portMAX_DELAY); }
    inline void m_unlock(SemaphoreHandle_t lock) const { xSemaphoreGive(lock); }

   private:
    ReplHistory _history;

    mutable SemaphoreHandle_t               _cmd_list_lock;
    std::forward_list<types::el_repl_cmd_t> _cmd_list;

    mutable SemaphoreHandle_t _exec_lock;

    types::el_repl_echo_cb_t _echo_cb;

    bool        _is_ctrl;
    std::string _ctrl_line;
    std::string _line;
    int         _line_index;
};

}  // namespace edgelab::repl

#endif
