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

#include <algorithm>
#include <cstring>
#include <deque>
#include <forward_list>
#include <locale>
#include <string>
#include <utility>

namespace edgelab {

ReplHistory::ReplHistory(int max_size) : _history_index(-1), _max_size(max_size) {}

el_err_code_t ReplHistory::add(const std::string& line) {
    if (line.empty()) [[unlikely]]
        return EL_OK;

    if (!_history.empty() && _history.back().compare(line) == 0) return EL_OK;

    auto it = std::find_if(_history.begin(), _history.end(), [&](const auto& l) { return l.compare(line) == 0; });

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

    if (_history_index >= 0) [[likely]] {
        line = _history[_history_index];
        --_history_index;
    }

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

el_err_code_t ReplServer::register_cmd(const el_repl_cmd_t& cmd) {
    if (cmd._cmd.empty()) [[unlikely]]
        return EL_EINVAL;

    _cmd_list.emplace_front(std::move(cmd));

    return EL_OK;
}

el_err_code_t ReplServer::register_cmd(const char* cmd, const char* desc, const char* arg, el_repl_cmd_cb_t cmd_cb) {
    el_repl_cmd_t cmd_t(cmd, desc, arg, cmd_cb);

    return register_cmd(std::move(cmd_t));
}

size_t ReplServer::register_cmds(const std::forward_list<el_repl_cmd_t>& cmd_list) {
    size_t registered_cmd_count = 0;
    for (const auto& cmd : cmd_list)
        if (register_cmd(cmd) == EL_OK) [[likely]]
            ++registered_cmd_count;

    return registered_cmd_count;
}

size_t ReplServer::register_cmds(const el_repl_cmd_t* cmd_list, size_t size) {
    size_t registered_cmd_count = 0;
    for (size_t i = 0; i < size; ++i)
        if (register_cmd(cmd_list[i]) == EL_OK) [[likely]]
            ++registered_cmd_count;

    return registered_cmd_count;
}

el_err_code_t ReplServer::unregister_cmd(const std::string& cmd) {
    _cmd_list.remove_if([&](const auto& c) { return c._cmd.compare(cmd) == 0; });

    return EL_OK;
}

el_err_code_t ReplServer::unregister_cmd(const char* cmd) {
    std::string cmd_str(cmd);

    return unregister_cmd(cmd_str);
}

el_err_code_t ReplServer::print_help() {
    el_printf("Command list:\n");
    for (const auto& cmd : _cmd_list) {
        if (cmd._args.size())
            el_printf("  AT+%s=<%s>\n", cmd._cmd.c_str(), cmd._args.c_str());
        else
            el_printf("  AT+%s\n", cmd._cmd.c_str());
        el_printf("    %s\n", cmd._desc.c_str());
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
                el_printf("\r> %s\033[K", _line.c_str());
            } else if (_ctrl_line.compare("[B") == 0) {
                _history.next(_line);
                _line_index = _line.size() - 1;
                el_printf("\r> %s\033[K", _line.c_str());
            } else if (_ctrl_line.compare("[C") == 0) {
                if (_line_index < _line.size() - 1) {
                    ++_line_index;
                    el_printf("\033%s", _ctrl_line.c_str());
                }
            } else if (_ctrl_line.compare("[D") == 0) {
                if (_line_index >= 0) {
                    --_line_index;
                    el_printf("\033%s", _ctrl_line.c_str());
                }
            } else if (_ctrl_line.compare("[H") == 0) {
                _line_index = 0;
                el_printf("\r\033[K> %s\033[%uG", _line.c_str(), _line_index + 3);
            } else if (_ctrl_line.compare("[F") == 0) {
                _line_index = _line.size() - 1;
                el_printf("\r\033[K> %s\033[%uG", _line.c_str(), _line_index + 4);
            } else if (_ctrl_line.compare("[3~") == 0) {
                if (_line_index < (_line.size() - 1)) {
                    if (!_line.empty() && _line_index >= 0) {
                        _line.erase(_line_index + 1, 1);
                        --_line_index;
                        el_printf("\r> %s\033[K\033[%uG", _line.c_str(), _line_index + 4);
                    }
                }
            } else
                el_printf("\033%s", _ctrl_line.c_str());

            _ctrl_line.clear();
            _is_ctrl = false;
        }

        return;
    }

    switch (c) {
    case '\n':
    case '\r':
        el_printf("\r\n");
        if (!_line.empty()) {
            if (m_exec_cmd(_line) == EL_OK) {
                _history.add(_line);
            }
            _line.clear();
            _line_index = -1;
        }
        _history.reset();
        el_printf("\r> ");
        break;

    case '\b':
    case 0x7F:
        if (!_line.empty() && _line_index >= 0) {
            _line.erase(_line_index, 1);
            --_line_index;
            el_printf("\r> %s\033[K\033[%uG", _line.c_str(), _line_index + 4);
        }
        break;

    case 0x1B:
        _is_ctrl = true;
        break;

    default:
        if (std::isprint(c)) {
            ++_line_index;
            _line.insert(_line_index, 1, c);
            if (_line_index == (_line.size() - 1))
                el_putchar(c);
            else
                el_printf("\r> %s\033[%uG", _line.c_str(), _line_index + 4);
        }
    }
}

el_err_code_t ReplServer::m_exec_cmd(const std::string& cmd) {
    el_err_code_t ret = EL_EINVAL;
    std::string   cmd_name;
    std::string   cmd_args;
    int           argc                            = 0;
    char*         argv[EL_CONFIG_AT_CMD_ARGC_MAX] = {};

    size_t pos = cmd.find_first_of("=");
    if (pos != std::string::npos) {
        cmd_name = cmd.substr(0, pos);
        cmd_args = cmd.substr(pos + 1);
    } else
        cmd_name = cmd;

    std::transform(cmd_name.begin(), cmd_name.end(), cmd_name.begin(), ::toupper);

    if (cmd_name.rfind("AT+", 0) != 0) {
        el_printf("Unknown command: %s\n", cmd_name.c_str());
        return EL_EINVAL;
    }

    cmd_name = cmd_name.substr(3);

    auto it = std::find_if(_cmd_list.begin(), _cmd_list.end(), [&](const auto& c) {
        size_t cmd_body_pos = cmd_name.rfind("|");
        return c._cmd.compare(cmd_name.substr(cmd_body_pos != std::string::npos ? cmd_body_pos + 1 : 0)) == 0;
    });

    if (it == _cmd_list.end()) [[unlikely]] {
        el_printf("Unknown command: %s\n", cmd_name.c_str());
        return ret;
    }

    argv[argc++] = const_cast<char*>(cmd_name.c_str());
    char* token  = std::strtok(const_cast<char*>(cmd_args.c_str()), ",");
    while (token && argc < EL_CONFIG_AT_CMD_ARGC_MAX) {
        argv[argc++] = token;
        token        = std::strtok(nullptr, ",");
    }

    if (it->_cmd_cb) {
        if (it->_argc != argc - 1) [[unlikely]] {
            el_printf("Command %s got wrong arguements.\n", cmd_name.c_str());
            return ret;
        }

        ret = it->_cmd_cb(argc, argv);
    }

    if (ret != EL_OK) [[unlikely]]
        el_printf("Command %s failed.\n", cmd_name.c_str());
    return ret;
}

ReplServer::ReplServer() : _is_ctrl(false), _line_index(-1) {
    register_cmd("HELP", "List available commands", "", [this](int, char**) -> el_err_code_t {
        this->print_help();
        return EL_OK;
    });
};

ReplServer::~ReplServer() { deinit(); }

void ReplServer::init() {
    _line.clear();

    el_printf("Welcome to EegeLab REPL.\n");
    el_printf("Type 'AT+HELP' for command list.\n");
    el_printf("> ");
}

void ReplServer::deinit() {
    _history.clear();
    _cmd_list.clear();

    _is_ctrl = false;
    _ctrl_line.clear();
    _line.clear();
    _line_index = -1;
}

}  // namespace edgelab
