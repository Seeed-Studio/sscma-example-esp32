#include "el_repl_server.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstring>
#include <forward_list>
#include <functional>
#include <string>

#include "el_compiler.h"
#include "el_debug.h"
#include "el_repl_history.hpp"
#include "el_types.h"

namespace edgelab::repl {

el_err_code_t ReplServer::register_cmd(const types::el_repl_cmd_t& cmd) {
    const Guard guard(this);  // TODO: avoid repeatily call when register_cmds

    if (cmd._cmd.empty()) [[unlikely]]
        return EL_EINVAL;
    _cmd_list.emplace_front(std::move(cmd));

    return EL_OK;
}

el_err_code_t ReplServer::register_cmd(const char*             cmd,
                                       const char*             desc,
                                       const char*             arg,
                                       types::el_repl_cmd_cb_t cmd_cb) {
    types::el_repl_cmd_t cmd_t(cmd, desc, arg, cmd_cb);

    return register_cmd(std::move(cmd_t));
}

size_t ReplServer::register_cmds(const std::forward_list<types::el_repl_cmd_t>& cmd_list) {
    size_t registered_cmd_count = 0;
    for (const auto& cmd : cmd_list)
        if (register_cmd(cmd) == EL_OK) [[likely]]
            ++registered_cmd_count;

    return registered_cmd_count;
}

size_t ReplServer::register_cmds(const types::el_repl_cmd_t* cmd_list, size_t size) {
    size_t registered_cmd_count = 0;
    for (size_t i = 0; i < size; ++i)
        if (register_cmd(cmd_list[i]) == EL_OK) [[likely]]
            ++registered_cmd_count;

    return registered_cmd_count;
}

el_err_code_t ReplServer::unregister_cmd(const std::string& cmd) {
    const Guard guard(this);

    _cmd_list.remove_if([&](const types::el_repl_cmd_t& c) { return c._cmd.compare(cmd) == 0; });

    return EL_OK;
}

el_err_code_t ReplServer::unregister_cmd(const char* cmd) {
    std::string cmd_str(cmd);

    return unregister_cmd(cmd_str);
}

el_err_code_t ReplServer::print_help() {
    const Guard guard(this);

    _echo_cb("Command list:\n");
    for (const auto& cmd : _cmd_list) {
        if (cmd._args.size())
            m_echo_cb("  AT+", cmd._cmd, "=<", cmd._args, ">\n");
        else
            m_echo_cb("  AT+", cmd._cmd, "\n");
        m_echo_cb("    ", cmd._desc, "\n");
    }

    return EL_OK;
}

void ReplServer::loop(const std::string& line) { loop(line.c_str(), line.size()); }

void ReplServer::loop(const char* line, size_t len) {
    for (size_t i = 0; i < len; ++i) loop(line[i]);
}

void ReplServer::loop(char c) {
    if (_is_ctrl) {
        _ctrl_line.push_back(c);
        if (std::isalpha(c) || (c == '~')) {
            if (_ctrl_line.compare("[A") == 0) {
                _history.prev(_line);
                _line_index = _line.size() - 1;
                m_echo_cb("\r> ", _line, "\033[K");
            } else if (_ctrl_line.compare("[B") == 0) {
                _history.next(_line);
                _line_index = _line.size() - 1;
                m_echo_cb("\r> ", _line, "\033[K");
            } else if (_ctrl_line.compare("[C") == 0) {
                if (_line_index < _line.size() - 1) {
                    ++_line_index;
                    m_echo_cb("\033", _ctrl_line);
                }
            } else if (_ctrl_line.compare("[D") == 0) {
                if (_line_index >= 0) {
                    --_line_index;
                    m_echo_cb("\033", _ctrl_line);
                }
            } else if (_ctrl_line.compare("[H") == 0) {
                _line_index = 0;
                m_echo_cb("\r\033[K> ", _line, "\033[", _line_index + 3, "G");
            } else if (_ctrl_line.compare("[F") == 0) {
                _line_index = _line.size() - 1;
                m_echo_cb("\r\033[K> ", _line, "\033[", _line_index + 4, "G");
            } else if (_ctrl_line.compare("[3~") == 0) {
                if (_line_index < (_line.size() - 1)) {
                    if (!_line.empty() && _line_index >= 0) {
                        _line.erase(_line_index + 1, 1);
                        --_line_index;
                        m_echo_cb("\r> ", _line, "\033[K\033[", _line_index + 4, "G");
                    }
                }
            } else
                m_echo_cb("\033", _ctrl_line);

            _ctrl_line.clear();
            _is_ctrl = false;
        }

        return;
    }

    switch (c) {
    case '\n':
    case '\r':
        _echo_cb("\r\n");
        if (!_line.empty()) {
            if (m_exec_cmd(_line) == EL_OK) {
                _history.add(_line);
            }
            _line.clear();
            _line_index = -1;
        }
        _history.reset();
        _echo_cb("\r> ");
        break;

    case '\b':
    case 0x7F:
        if (!_line.empty() && _line_index >= 0) {
            _line.erase(_line_index, 1);
            --_line_index;
            m_echo_cb("\r> ", _line, "\033[K\033[", _line_index + 4, "G");
        }
        break;

    case 0x1B:
        _is_ctrl = true;
        break;

    default:
        if (std::isprint(c)) {
            _line.insert(++_line_index, 1, c);
            if (_line_index == (_line.size() - 1))
                el_putchar(c);
            else
                m_echo_cb("\r> ", _line, "\033[", _line_index + 4, "G");
        }
    }
}

el_err_code_t ReplServer::m_exec_cmd(const std::string& cmd) {
    el_err_code_t ret = EL_EINVAL;
    std::string   cmd_name;
    std::string   cmd_args;
    int           argc                              = 0;
    char*         argv[CONFIG_EL_REPL_CMD_ARGC_MAX] = {};

    size_t pos = cmd.find_first_of("=");
    if (pos != std::string::npos) {
        cmd_name = cmd.substr(0, pos);
        cmd_args = cmd.substr(pos + 1);
    } else
        cmd_name = cmd;

    std::transform(cmd_name.begin(), cmd_name.end(), cmd_name.begin(), ::toupper);

    if (cmd_name.rfind("AT+", 0) != 0) {
        m_echo_cb("Unknown command: ", cmd, "\n");
        return EL_EINVAL;
    }

    cmd_name = cmd_name.substr(3);

    m_lock();
    auto it = std::find_if(_cmd_list.begin(), _cmd_list.end(), [&](const types::el_repl_cmd_t& c) {
        size_t cmd_body_pos = cmd_name.rfind("|");
        return c._cmd.compare(cmd_name.substr(cmd_body_pos != std::string::npos ? cmd_body_pos + 1 : 0)) == 0;
    });
    if (it == _cmd_list.end()) [[unlikely]] {
        m_echo_cb("Unknown command: ", cmd, "\n");
        m_unlock();
        return ret;
    }
    auto cmd_copy = *it;
    m_unlock();

    argv[argc++] = const_cast<char*>(cmd_name.c_str());
    char* token  = std::strtok(const_cast<char*>(cmd_args.c_str()), ",");
    while (token && argc < CONFIG_EL_REPL_CMD_ARGC_MAX) {
        argv[argc++] = token;
        token        = std::strtok(nullptr, ",");
    }

    if (cmd_copy._cmd_cb) {
        if (cmd_copy._argc != argc - 1) [[unlikely]] {
            m_echo_cb("Command ", cmd_name, " got wrong arguements.\n");
            return ret;
        }
        ret = cmd_copy._cmd_cb(argc, argv);
    }

    if (ret != EL_OK) [[unlikely]]
        m_echo_cb("Command ", cmd_name, " failed.\n");
    return ret;
}

ReplServer::ReplServer() : _cmd_list_lock(xSemaphoreCreateCounting(1, 1)), _is_ctrl(false), _line_index(-1) {
    register_cmd("HELP", "List available commands", "", [this](int, char**) -> el_err_code_t {
        this->print_help();
        return EL_OK;
    });
};

ReplServer::~ReplServer() { deinit(); }

void ReplServer::init(types::el_repl_echo_cb_t echo_cb) {
    {
        const Guard guard(this);
        EL_ASSERT(echo_cb);
        _echo_cb = echo_cb;
    }

    m_echo_cb("Welcome to EegeLab REPL.\n", "Type 'AT+HELP' for command list.\n", "> ");
}

void ReplServer::deinit() {
    const Guard guard(this);

    _history.clear();
    _cmd_list.clear();

    _is_ctrl = false;
    _ctrl_line.clear();
    _line.clear();
    _line_index = -1;
}

}  // namespace edgelab::repl
