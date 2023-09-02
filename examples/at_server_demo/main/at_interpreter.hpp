#pragma once

#include <functional>
#include <locale>
#include <string>

namespace intr::types {

static const char lparn = '(';
static const char rparn = ')';

enum class TokenType { IDENTIFIER, CONSTANT, OPERATOR, LPARN, RPARN, FUNCTION, TERM };

struct Token {
    TokenType   type;
    std::string value;
};

enum class NodeType { IDENTIFIER, FUNCTION_CALL };

typedef std::function<int(NodeType, const std::string&)> EvalCbType;

class ASTNode {
   public:
    virtual ~ASTNode() = default;

    virtual int evaluate(EvalCbType callback) const = 0;
};

}  // namespace intr::types

namespace intr::utility {

using namespace intr::types;

bool is_identifier(char c) { return std::isalnum(c) || c == '_'; }

bool is_constant(char c) { return std::isdigit(c); }

bool is_arithmetic_operator(char c) { return c == '*' || c == '/' || c == '+' || c == '-'; }

bool is_relational_operator_pred(char c) { return c == '<' || c == '>' || c == '=' || c == '!'; }

bool is_relational_operator_term(char c) { return c == '='; }

bool is_logical_operator(char c) { return c == '&' || c == '|'; }

bool is_lparn(char c) { return c == lparn; }

bool is_rparn(char c) { return c == rparn; }

bool is_comma(char c) { return c == ','; }

}  // namespace intr::utility

namespace intr {

using namespace intr::types;

class IdentifierNode : public ASTNode {
   public:
    IdentifierNode(const std::string& identifier_name) : _identifier_name(identifier_name) {}

    ~IdentifierNode() = default;

    int evaluate(EvalCbType callback) const override { return callback(NodeType::IDENTIFIER, _identifier_name); }

   private:
    std::string _identifier_name;
};

class ConstantNode : public ASTNode {
   public:
    ConstantNode(const std::string& value) : _value(std::atoi(value.c_str())) {}

    ~ConstantNode() = default;

    int evaluate(EvalCbType) const override { return _value; }

   private:
    int _value;
};

class BinaryOperatorNode : public ASTNode {
   public:
    BinaryOperatorNode(const std::string& operator_name, ASTNode* left, ASTNode* right)
        : _operator_name(operator_name), _left(left), _right(right) {}

    ~BinaryOperatorNode() {
        if (_left) delete _left;
        if (_right) delete _right;
    }

    int evaluate(EvalCbType callback) const override {
        int left_value  = _left->evaluate(callback);
        int right_value = _right->evaluate(callback);

        if (_operator_name == "*") return left_value * right_value;

        if (_operator_name == "/") return left_value / right_value;

        if (_operator_name == "+") return left_value + right_value;

        if (_operator_name == "-") return left_value - right_value;

        if (_operator_name == ">") return left_value > right_value;

        if (_operator_name == "<") return left_value < right_value;

        if (_operator_name == ">=") return left_value >= right_value;

        if (_operator_name == "<=") return left_value <= right_value;

        if (_operator_name == "==") return left_value == right_value;

        if (_operator_name == "!=") return left_value != right_value;

        if (_operator_name == "&&") return left_value && right_value;

        if (_operator_name == "||") return left_value || right_value;

        // skip error
        return 0;
    }

   private:
    std::string _operator_name;
    ASTNode*    _left;
    ASTNode*    _right;
};

class FunctionCallNode : public ASTNode {
   public:
    FunctionCallNode(const std::string& function_call) : _function_call(function_call) {}

    ~FunctionCallNode() = default;

    int evaluate(EvalCbType callback) const override { return callback(NodeType::FUNCTION_CALL, _function_call); }

   private:
    std::string _function_call;
};

}  // namespace intr

#include <cstdint>
#include <stack>
#include <string>
#include <utility>

namespace intr {

using namespace intr::types;
using namespace intr::utility;

class Lexer {
   public:
    explicit Lexer(const std::string& input) noexcept : _input(input), _length(input.size() - 1), _index(0) {
        _current_char = _input.at(_index);
    }

    ~Lexer() = default;

    Token get_next_token() noexcept {
        do {
            if (std::isalpha(_current_char)) {
                std::string identifier{get_identifier()};
                if (is_lparn(_current_char)) {
                    identifier += get_function_parms();
                    return Token{.type = TokenType::FUNCTION, .value = std::move(identifier)};
                }
                return Token{.type = TokenType::IDENTIFIER, .value = std::move(identifier)};
            }

            if (std::isdigit(_current_char)) {
                return Token{.type = TokenType::CONSTANT, .value = get_constant()};
            }

            if (is_lparn(_current_char)) {
                size_t head = _index;
                advance();
                return Token{.type = TokenType::LPARN, .value = _input.substr(head, 1)};
            }

            if (is_rparn(_current_char)) {
                size_t head = _index;
                advance();
                return Token{.type = TokenType::RPARN, .value = _input.substr(head, 1)};
            }

            if (is_arithmetic_operator(_current_char)) {
                size_t head = _index;
                advance();
                return Token{.type = TokenType::OPERATOR, .value = _input.substr(head, 1)};
            }

            if (is_logical_operator(_current_char)) {
                size_t head = _index;
                if (advance() && is_logical_operator(_current_char)) {
                    return Token{.type = TokenType::OPERATOR, .value = _input.substr(head, _index - head)};
                }
                break;  // skip error
            }

            if (is_relational_operator_pred(_current_char)) {
                size_t head = _index;
                if (advance() && is_relational_operator_term(_current_char)) advance();
                return Token{.type = TokenType::OPERATOR, .value = _input.substr(head, _index - head)};
            }

        } while (advance());

        return Token{.type = TokenType::TERM, .value = std::string{}};
    }

   protected:
    inline bool advance() noexcept {
        if (_index++ < _length) [[likely]] {
            _current_char = _input.at(_index);
            return true;
        } else {
            _current_char = '\0';
            return false;
        }
    }

    inline std::string get_constant() noexcept {
        size_t head = _index;

        while (advance() && is_constant(_current_char))
            ;

        return _input.substr(head, _index - head);
    }

    inline std::string get_identifier() noexcept {
        size_t head = _index;

        while (advance() && is_identifier(_current_char))
            ;

        return _input.substr(head, _index - head);
    }

    inline std::string get_function_parms() noexcept {
        size_t head = _index;

        std::stack<char> stk;
        stk.push(rparn);

        while (advance() && stk.size()) {
            if (_current_char == stk.top())
                stk.pop();
            else if (is_lparn(_current_char))
                stk.push(rparn);
        }

        return _input.substr(head, _index - head);
    }

   private:
    const std::string& _input;
    size_t             _length;
    size_t             _index;
    char               _current_char;
};

}  // namespace intr

#include <forward_list>
#include <stack>

namespace intr {

using namespace intr::types;

class Parser {
   public:
    Parser(Lexer& lexer) : _lexer(lexer), _current_token(lexer.get_next_token()) {}

    ~Parser() = default;

    ASTNode* parse() {
        while (_current_token.type != TokenType::TERM && _node_stack.size() < 3 && parse_expression()) {
            _current_token = _lexer.get_next_token();
        }

        if (_node_stack.size() != 1) {
            while (_node_stack.size()) {
                delete _node_stack.top();
                _node_stack.pop();
            }
            return nullptr;
        }

        return _node_stack.top();
    }

    const std::forward_list<std::string>& get_mutable() { return _mutable; }

   protected:
    bool parse_expression() {
        if (_current_token.type == TokenType::IDENTIFIER) {
            _mutable.emplace_front(_current_token.value);
            _node_stack.push(new IdentifierNode(_current_token.value));
            return true;
        }

        if (_current_token.type == TokenType::CONSTANT) {
            _node_stack.push(new ConstantNode(_current_token.value));
            return true;
        }

        if (_current_token.type == TokenType::FUNCTION) {
            _mutable.emplace_front(_current_token.value);
            _node_stack.push(new FunctionCallNode(_current_token.value));
            return true;
        }

        if (_current_token.type == TokenType::OPERATOR) {
            Token operator_token = _current_token;
            if (_node_stack.size() == 1) {
                _current_token = _lexer.get_next_token();
                if (!parse_expression()) [[unlikely]]
                    return false;
            }

            if (_node_stack.size() != 2) return false;

            ASTNode* right = _node_stack.top();
            _node_stack.pop();
            ASTNode* left = _node_stack.top();
            _node_stack.pop();

            _node_stack.push(new BinaryOperatorNode(operator_token.value, left, right));

            return true;
        }

        if (_current_token.type == TokenType::LPARN) {
            _node_stack.push(Parser(_lexer).parse());
            return true;
        }

        if (_current_token.type == TokenType::RPARN) {
            return false;
        }

        // skip error
        return false;
    }

   private:
    Lexer&                         _lexer;
    Token                          _current_token;
    std::stack<ASTNode*>           _node_stack;
    std::forward_list<std::string> _mutable;
};

}  // namespace intr
