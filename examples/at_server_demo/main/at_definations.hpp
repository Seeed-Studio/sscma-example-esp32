#pragma once

#include <sstream>
#include <functional>

#define kTensorArenaSize (1024 * 1024)

#define REPLY_CMD_HEADER "\r{\"type\": 0, "
#define REPLY_EVT_HEADER "\r{\"type\": 1, "
#define REPLY_LOG_HEADER "\r{\"type\": 2, "

using DELIM_F                = std::function<void(std::ostringstream& os)>;
static DELIM_F delim_f       = [](std::ostringstream& os) {};
static DELIM_F print_delim_f = [](std::ostringstream& os) { os << ", "; };
static DELIM_F print_void_f  = [](std::ostringstream& os) { delim_f = print_delim_f; };

#define DELIM_RESET \
    { delim_f = print_void_f; }
#define DELIM_PRINT(OS) \
    { delim_f(OS); }
