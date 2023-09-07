#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <functional>
#include <string>
#include <unordered_map>

#include "at_interpreter.hpp"

typedef std::function<void(void)>                     branch_cb_t;
typedef std::function<int(void)>                      mutable_cb_t;
typedef std::unordered_map<std::string, mutable_cb_t> mutable_map_t;

class ActionDelegate {
   public:
    static ActionDelegate* get_delegate() {
        static auto action_delegate = ActionDelegate();
        return &action_delegate;
    }

    ~ActionDelegate() { unset_condition(); }

    bool has_condition() {
        const Guard guard(this);

        return _node ? true : false;
    }

    bool set_condition(const std::string& input) {
        const Guard guard(this);

        intr::Lexer    lexer(input);
        intr::Mutables mutables;
        intr::Parser   parser(lexer, mutables);

        _node = parser.parse();

        if (!_node) [[unlikely]]
            return false;

        for (const auto& key : mutables) _mutable_map[key] = nullptr;

        return true;
    }

    const mutable_map_t& get_mutable_map() {
        const Guard guard(this);

        return _mutable_map;
    }

    void set_mutable_map(const mutable_map_t& map) {
        const Guard guard(this);

        _mutable_map = map;
    }

    void set_true_cb(branch_cb_t cb) {
        const Guard guard(this);

        _true_cb = cb;
    }

    void set_false_exception_cb(branch_cb_t cb) {
        const Guard guard(this);

        _false_or_exception_cb = cb;
    }

    bool evalute() {
        const Guard guard(this);

        if (!_node) [[unlikely]]
            return false;
        if (!_true_cb) [[unlikely]]
            return false;
        if (!_false_or_exception_cb) [[unlikely]]
            return false;

        if (_node->evaluate([this](intr::types::NodeType, const std::string& name) {
                auto it = this->_mutable_map.find(name);
                if (it != this->_mutable_map.end() && it->second) [[likely]]
                    return it->second();
                return 0;
            }))
            _true_cb();
        else
            _false_or_exception_cb();

        return true;
    }

    void unset_condition() {
        const Guard guard(this);

        if (_node) [[likely]] {
            delete _node;
            _node = nullptr;
        }
        _mutable_map.clear();
    }

   protected:
    ActionDelegate() : _node(nullptr), _eval_lock(xSemaphoreCreateCounting(1, 1)){};

    inline void m_lock() const noexcept { xSemaphoreTake(_eval_lock, portMAX_DELAY); }
    inline void m_unlock() const noexcept { xSemaphoreGive(_eval_lock); }

    struct Guard {
        Guard(const ActionDelegate* const action) noexcept : __action(action) { __action->m_lock(); }
        ~Guard() noexcept { __action->m_unlock(); }

        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;

       private:
        const ActionDelegate* const __action;
    };

   private:
    intr::ASTNode* _node;

    SemaphoreHandle_t _eval_lock;

    mutable_map_t _mutable_map;

    branch_cb_t _true_cb;
    branch_cb_t _false_or_exception_cb;
};
