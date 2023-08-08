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

#include "el_repl.h"

#include <algorithm>

namespace edgelab {

ReplServer* ReplServer::_instance = nullptr;

el_err_code_t ReplHistory::add(std::string& line) {
    if (line.empty()) {
        return EL_OK;
    }

    if (!_history.empty() && _history.back() == line) {
        return EL_OK;
    }

    for (auto it = _history.begin(); it != _history.end(); it++) {
        if (*it == line) {
            _history.erase(it);
            break;
        }
    }

    if (_history.size() >= _max_size) {
        _history.pop_front();
    }

    _history.push_back(line);
    _history_index = _history.size() - 1;

    return EL_OK;
}

el_err_code_t ReplHistory::get(std::string& line, int index) {
    if (index < 0 || index >= _history.size()) {
        return EL_ELOG;
    }

    line = _history[index];
    return EL_OK;
}

el_err_code_t ReplHistory::next(std::string& line) {
    if (_history.empty()) {
        return EL_ELOG;
    }

    if (_history_index < 0) {
        _history_index = _history.size() - 1;
    }

    if (_history_index != _history.size() - 1) {
        _history_index++;
    }

    line = _history[_history_index];

    return EL_OK;
}

el_err_code_t ReplHistory::prev(std::string& line) {
    if (_history.empty()) {
        return EL_ELOG;
    }

    line = _history[_history_index];

    if (_history_index != 0) {
        _history_index--;
    }

    return EL_OK;
}

el_err_code_t ReplHistory::clear() {
    _history.clear();
    _history_index = -1;
    return EL_OK;
}

void ReplHistory::print() {
    for (auto& line : _history) el_printf("%s\n", line.c_str());
}

el_err_code_t ReplServer::register_cmd(el_repl_cmd_t& cmd) {
    if (cmd.cmd.empty()) return EL_ELOG;

    std::transform(cmd.cmd.begin(), cmd.cmd.end(), cmd.cmd.begin(), ::toupper);

    _cmd_list.push_back(cmd);
    return EL_OK;
}

el_err_code_t ReplServer::register_cmd(const char*            cmd,
                                const char*            desc,
                                const char*            arg,
                                el_repl_cmd_exec_cb_t  exec_cb,
                                el_repl_cmd_read_cb_t  read_cb,
                                el_repl_cmd_write_cb_t write_cb) {
    el_repl_cmd_t cmd_t;
    cmd_t.cmd  = cmd;
    cmd_t.desc = desc;
    if (arg) cmd_t.arg = arg;
    cmd_t.exec_cb  = exec_cb;
    cmd_t.read_cb  = read_cb;
    cmd_t.write_cb = write_cb;

    return register_cmd(cmd_t);
}

el_err_code_t ReplServer::register_cmds(std::vector<el_repl_cmd_t>& cmd_list) {
    for (auto& cmd : cmd_list) {
        register_cmd(cmd);
    }

    return EL_OK;
}

el_err_code_t ReplServer::register_cmds(el_repl_cmd_t* cmd_list, int size) {
    for (int i = 0; i < size; i++) {
        register_cmd(cmd_list[i]);
    }

    return EL_OK;
}

el_err_code_t ReplServer::unregister_cmd(std::string& cmd) {
    for (auto it = _cmd_list.begin(); it != _cmd_list.end(); it++) {
        if (it->cmd == cmd) {
            _cmd_list.erase(it);
            return EL_OK;
        }
    }

    return EL_ELOG;
}

el_err_code_t ReplServer::print_help() {
    el_printf("Command list:\n");
    for (auto& cmd : _cmd_list) {
        if (cmd.exec_cb) {
            el_printf("  AT+%s\n", cmd.cmd.c_str());
        } else if (cmd.read_cb) {
            el_printf("  AT+%s?\n", cmd.cmd.c_str());
        } else if (cmd.write_cb) {
            el_printf("  AT+%s=<%s>\n", cmd.cmd.c_str(), cmd.arg.c_str());
        }
        el_printf("    %s\n\n", cmd.desc.c_str());
    }
    return EL_OK;
}

void ReplServer::loop(std::string& line) { loop(line.c_str(), line.size()); }

void ReplServer::loop(const char* line, size_t len) {
    for (int i = 0; i < len; i++) {
        loop(line[i]);
    }
}

void ReplServer::loop(char c) {
    if (_is_ctrl) {
        _ctrl_line.push_back(c);
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c == '~')) {
            if (_ctrl_line == "[A") {  // Up
                _history.prev(_line);
                _line_index = _line.size() - 1;
                el_printf("\r> %s\033[K", _line.c_str());
            } else if (_ctrl_line == "[B") {  // Down
                _history.next(_line);
                _line_index = _line.size() - 1;
                el_printf("\r> %s\033[K", _line.c_str());
            } else if (_ctrl_line == "[C") {  // Right
                if (_line_index < (int)(_line.size() - 1)) {
                    _line_index++;
                    el_printf("\033%s", _ctrl_line.c_str());
                }
            } else if (_ctrl_line == "[D") {  // Left
                if (_line_index >= 0) {
                    _line_index--;
                    el_printf("\033%s", _ctrl_line.c_str());
                }
            } else if (_ctrl_line == "[H")  // home
            {
                _line_index = 0;
                el_printf("\r\033[K> %s\033[%uG", _line.c_str(), _line_index + 3);
            } else if (_ctrl_line == "[F")  // end
            {
                _line_index = _line.size() - 1;
                el_printf("\r\033[K> %s\033[%uG", _line.c_str(), _line_index + 4);
            } else if (_ctrl_line == "[3~")  // delete
            {
                if (_line_index < (int)(_line.size() - 1)) {
                    if (!_line.empty() && _line_index >= 0) {
                        _line.erase(_line_index + 1, 1);
                        _line_index--;
                        el_printf("\r> %s\033[K\033[%uG", _line.c_str(), _line_index + 4);
                    }
                }
            } else {
                el_printf("\033%s", _ctrl_line.c_str());
            }
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
            if (_exec_cmd(_line) == EL_OK) {
                _history.add(_line);
            }
            _line.clear();
            _line_index = -1;
        }
        el_printf("\r> ");
        break;
    case '\b':  // Backspace
    case 0x7f:  // Delete
        if (!_line.empty() && _line_index >= 0) {
            _line.erase(_line_index, 1);
            _line_index--;
            el_printf("\r> %s\033[K\033[%uG", _line.c_str(), _line_index + 4);
        }
        break;
    case 0x1b:  // Escape
        _is_ctrl = true;
        break;

    default:
        if (c >= 0x20 && c <= 0x7e) {
            _line_index++;
            _line.insert(_line_index, 1, c);
            if (_line_index == (int)(_line.size() - 1)) {
                el_putchar(c);
            } else {
                el_printf("\r> %s\033[%uG", _line.c_str(), _line_index + 4);
            }
        }
        break;
    }
}

el_err_code_t ReplServer::_exec_cmd(std::string& cmd) {
    el_err_code_t             ret      = EL_ELOG;
    el_repl_cmd_type_t cmd_type = EL_REPL_CMD_NONE;
    std::string        cmd_name;
    std::string        cmd_arg;
    char*              token    = NULL;
    int                argc     = 0;
    char*              argv[10] = {NULL};
    size_t             pos      = cmd.find_first_of("?=");

    if (pos != std::string::npos) {
        cmd_name = cmd.substr(0, pos);
        cmd_arg  = cmd.substr(pos + 1);
        if (cmd[pos] == '?') {
            cmd_type = EL_REPL_CMD_READ;
        } else if (cmd[pos] == '=') {
            cmd_type = EL_REPL_CMD_WRITE;
        }
    } else {
        cmd_name = cmd;
        cmd_type = EL_REPL_CMD_EXEC;
    }

    std::transform(cmd_name.begin(), cmd_name.end(), cmd_name.begin(), ::toupper);

    if (cmd_name.compare("AT") == 0) {
        el_printf("{\"at\": %ld, \"timestamp\": %lld}\n", unsigned(EL_OK), el_get_time_ms());
        return EL_OK;
    }

    if (cmd_name.compare("HELP") == 0) {
        print_help();
        return EL_OK;
    }

    if (cmd_name.rfind("AT+", 0) != 0) {
        el_printf("Unknown command: %s\n", cmd_name.c_str());
        return EL_ELOG;
    }

    cmd_name = cmd_name.substr(3);

    if (cmd_name == "HELP" && cmd_type == EL_REPL_CMD_EXEC) {
        print_help();
        return EL_OK;
    }

    if (_cmd_list.empty()) {
        el_printf("Unknown command: %s\n", cmd_name.c_str());
        return EL_ELOG;
    }

    do {
        token = strtok(argc == 0 ? (char*)cmd_arg.c_str() : NULL, ",");
        if (token) {
            argv[argc++] = token;
        }
    } while (token != NULL);

    for (auto& repl_cmd : _cmd_list) {
        if (repl_cmd.cmd == cmd_name) {
            if (cmd_type == EL_REPL_CMD_READ && repl_cmd.read_cb) {
                ret = repl_cmd.read_cb();
            } else if (cmd_type == EL_REPL_CMD_WRITE && repl_cmd.write_cb) {
                ret = repl_cmd.write_cb(argc, argv);
            } else if (cmd_type == EL_REPL_CMD_EXEC && repl_cmd.exec_cb) {
                ret = repl_cmd.exec_cb();
            }
            if (ret != EL_OK) {
                el_printf("Command %s failed.\n", cmd_name.c_str());
            }
            return ret;
        }
    }

    el_printf("Unknown command: %s\n", cmd_name.c_str());

    return ret;
}

void ReplServer::init() {
    _line.clear();

    el_printf("Welcome to EegeLab REPL.\n");
    el_printf("Type 'AT+HELP' for command list.\n");
    el_printf("> ");
}

}  // namespace edgelab