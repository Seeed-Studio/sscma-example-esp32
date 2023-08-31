#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "at_interpreter.hpp"

typedef std::function<void(void)>                     branch_cb_t;
typedef std::function<int(void)>                      mutable_cb_t;
typedef std::unordered_map<std::string, mutable_cb_t> mutable_map_t;

class EventDelegate {
   public:
    static EventDelegate* get_delegate() {
        static EventDelegate event_delegate = EventDelegate();
        return &event_delegate;
    }

    ~EventDelegate() { unset_condition(); }

    bool set_condition(const std::string& input) {
        intr::Lexer  lexer(input);
        intr::Parser parser(lexer);

        _node = parser.parse();

        if (!_node) [[unlikely]]
            return false;

        const auto& mutable_keys = parser.get_mutable();
        for (const auto& key : mutable_keys) _mutable_map[key] = nullptr;

        return _node != nullptr;
    }

    const mutable_map_t& get_mutable_map() { return _mutable_map; }

    void set_mutable_map(const mutable_map_t& map) { _mutable_map = map; }

    void set_true_cb(branch_cb_t cb) { _true_cb = cb; }

    void set_false_exception_cb(branch_cb_t cb) { _false_or_exception_cb = cb; }

    bool evalute() {
        if (!_node) [[unlikely]]
            return false;
        if (!_true_cb) [[unlikely]]
            return false;
        if (!_false_or_exception_cb) [[unlikely]]
            return false;

        if (_node->evaluate([this](intr::types::NodeType, const std::string& name) {
                auto it = this->_mutable_map.find(name);
                if (it != this->_mutable_map.end()) [[likely]]
                    return it->second();
                return 0;
            }))
            _true_cb();
        else
            _false_or_exception_cb();

        return true;
    }

    void unset_condition() {
        if (_node) [[likely]] {
            delete _node;
            _node = nullptr;
        }
        _mutable_map.clear();
    }

   protected:
    EventDelegate() : _node(nullptr){};

   private:
    intr::ASTNode* _node;

    mutable_map_t _mutable_map;

    branch_cb_t _true_cb;
    branch_cb_t _false_or_exception_cb;
};
