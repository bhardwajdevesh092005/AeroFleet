#pragma once

#include <string>
#include <vector>
#include <map>
#include <variant>
#include <stdexcept>
#include <sstream>
#include <cctype>
#include <fstream>

namespace aerofleet {

class JsonValue;
using JsonObject = std::map<std::string, JsonValue>;
using JsonArray = std::vector<JsonValue>;

class JsonParseException : public std::runtime_error {
public:
    explicit JsonParseException(const std::string& message) : std::runtime_error(message) {}
};

class JsonValue {
public:
    enum class Type { Null, Boolean, Number, String, Array, Object };

    using ValueData = std::variant<
        std::nullptr_t,
        bool,
        double,
        std::string,
        JsonArray,
        JsonObject
    >;

    JsonValue() : m_data(nullptr) {}
    JsonValue(std::nullptr_t) : m_data(nullptr) {}
    JsonValue(bool b) : m_data(b) {}
    JsonValue(double n) : m_data(n) {}
    JsonValue(int n) : m_data(static_cast<double>(n)) {}
    JsonValue(int64_t n) : m_data(static_cast<double>(n)) {}
    JsonValue(const std::string& s) : m_data(s) {}
    JsonValue(const char* s) : m_data(std::string(s)) {}
    JsonValue(JsonArray arr) : m_data(std::move(arr)) {}
    JsonValue(JsonObject obj) : m_data(std::move(obj)) {}

    [[nodiscard]] Type getType() const {
        switch (m_data.index()) {
            case 0: return Type::Null;
            case 1: return Type::Boolean;
            case 2: return Type::Number;
            case 3: return Type::String;
            case 4: return Type::Array;
            case 5: return Type::Object;
            default: return Type::Null;
        }
    }

    [[nodiscard]] bool isNull() const { return getType() == Type::Null; }
    [[nodiscard]] bool isBool() const { return getType() == Type::Boolean; }
    [[nodiscard]] bool isNumber() const { return getType() == Type::Number; }
    [[nodiscard]] bool isString() const { return getType() == Type::String; }
    [[nodiscard]] bool isArray() const { return getType() == Type::Array; }
    [[nodiscard]] bool isObject() const { return getType() == Type::Object; }

    [[nodiscard]] bool asBool() const {
        if (!isBool()) throw JsonParseException("Expected boolean JSON value");
        return std::get<bool>(m_data);
    }

    [[nodiscard]] double asDouble() const {
        if (!isNumber()) throw JsonParseException("Expected numeric JSON value");
        return std::get<double>(m_data);
    }

    [[nodiscard]] int64_t asInt() const {
        return static_cast<int64_t>(asDouble());
    }

    [[nodiscard]] const std::string& asString() const {
        if (!isString()) throw JsonParseException("Expected string JSON value");
        return std::get<std::string>(m_data);
    }

    [[nodiscard]] const JsonArray& asArray() const {
        if (!isArray()) throw JsonParseException("Expected array JSON value");
        return std::get<JsonArray>(m_data);
    }

    [[nodiscard]] const JsonObject& asObject() const {
        if (!isObject()) throw JsonParseException("Expected object JSON value");
        return std::get<JsonObject>(m_data);
    }

    [[nodiscard]] bool contains(const std::string& key) const {
        if (!isObject()) return false;
        const auto& obj = std::get<JsonObject>(m_data);
        return obj.find(key) != obj.end();
    }

    [[nodiscard]] const JsonValue& operator[](const std::string& key) const {
        if (!isObject()) throw JsonParseException("Indexing with key requires JSON object");
        const auto& obj = std::get<JsonObject>(m_data);
        auto it = obj.find(key);
        if (it == obj.end()) {
            throw JsonParseException("JSON key not found: " + key);
        }
        return it->second;
    }

    [[nodiscard]] const JsonValue& operator[](size_t index) const {
        if (!isArray()) throw JsonParseException("Indexing with integer requires JSON array");
        const auto& arr = std::get<JsonArray>(m_data);
        if (index >= arr.size()) {
            throw JsonParseException("JSON array index out of bounds");
        }
        return arr[index];
    }

    template<typename T>
    [[nodiscard]] T getOrDefault(const std::string& key, const T& defaultValue) const {
        if (!isObject()) return defaultValue;
        const auto& obj = std::get<JsonObject>(m_data);
        auto it = obj.find(key);
        if (it == obj.end()) return defaultValue;

        if constexpr (std::is_same_v<T, std::string>) {
            if (it->second.isString()) return it->second.asString();
        } else if constexpr (std::is_floating_point_v<T>) {
            if (it->second.isNumber()) return static_cast<T>(it->second.asDouble());
        } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
            if (it->second.isNumber()) return static_cast<T>(it->second.asInt());
        } else if constexpr (std::is_same_v<T, bool>) {
            if (it->second.isBool()) return it->second.asBool();
        }
        return defaultValue;
    }

    // Static parser
    static JsonValue parse(std::string_view jsonStr) {
        size_t idx = 0;
        skipWhitespace(jsonStr, idx);
        JsonValue val = parseValue(jsonStr, idx);
        skipWhitespace(jsonStr, idx);
        if (idx < jsonStr.size()) {
            throw JsonParseException("Unexpected trailing characters after JSON root at position " + std::to_string(idx));
        }
        return val;
    }

    static JsonValue fromFile(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw JsonParseException("Failed to open JSON file: " + filepath);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parse(buffer.str());
    }

private:
    ValueData m_data;

    static void skipWhitespace(std::string_view s, size_t& idx) {
        while (idx < s.size()) {
            char c = s[idx];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                idx++;
            } else if (c == '/' && idx + 1 < s.size() && s[idx + 1] == '/') {
                // Support line comments
                idx += 2;
                while (idx < s.size() && s[idx] != '\n') idx++;
            } else {
                break;
            }
        }
    }

    static JsonValue parseValue(std::string_view s, size_t& idx) {
        skipWhitespace(s, idx);
        if (idx >= s.size()) throw JsonParseException("Unexpected end of JSON input");

        char c = s[idx];
        if (c == '{') return parseObject(s, idx);
        if (c == '[') return parseArray(s, idx);
        if (c == '"') return parseString(s, idx);
        if (c == 't' || c == 'f') return parseBool(s, idx);
        if (c == 'n') return parseNull(s, idx);
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parseNumber(s, idx);

        throw JsonParseException(std::string("Unexpected character in JSON: '") + c + "' at position " + std::to_string(idx));
    }

    static JsonObject parseObject(std::string_view s, size_t& idx) {
        JsonObject obj;
        idx++; // skip '{'
        skipWhitespace(s, idx);

        if (idx < s.size() && s[idx] == '}') {
            idx++;
            return obj;
        }

        while (idx < s.size()) {
            skipWhitespace(s, idx);
            if (idx >= s.size() || s[idx] != '"') {
                throw JsonParseException("Expected string key in object at position " + std::to_string(idx));
            }
            std::string key = parseRawString(s, idx);

            skipWhitespace(s, idx);
            if (idx >= s.size() || s[idx] != ':') {
                throw JsonParseException("Expected ':' after key in object at position " + std::to_string(idx));
            }
            idx++; // skip ':'

            JsonValue val = parseValue(s, idx);
            obj[std::move(key)] = std::move(val);

            skipWhitespace(s, idx);
            if (idx >= s.size()) throw JsonParseException("Unterminated object");
            if (s[idx] == '}') {
                idx++;
                return obj;
            }
            if (s[idx] == ',') {
                idx++;
                continue;
            }
            throw JsonParseException("Expected ',' or '}' in object at position " + std::to_string(idx));
        }
        throw JsonParseException("Unterminated object at end of input");
    }

    static JsonArray parseArray(std::string_view s, size_t& idx) {
        JsonArray arr;
        idx++; // skip '['
        skipWhitespace(s, idx);

        if (idx < s.size() && s[idx] == ']') {
            idx++;
            return arr;
        }

        while (idx < s.size()) {
            arr.push_back(parseValue(s, idx));
            skipWhitespace(s, idx);
            if (idx >= s.size()) throw JsonParseException("Unterminated array");
            if (s[idx] == ']') {
                idx++;
                return arr;
            }
            if (s[idx] == ',') {
                idx++;
                continue;
            }
            throw JsonParseException("Expected ',' or ']' in array at position " + std::to_string(idx));
        }
        throw JsonParseException("Unterminated array at end of input");
    }

    static std::string parseRawString(std::string_view s, size_t& idx) {
        idx++; // skip opening '"'
        std::string result;
        while (idx < s.size()) {
            char c = s[idx++];
            if (c == '"') return result;
            if (c == '\\') {
                if (idx >= s.size()) throw JsonParseException("Unterminated escape sequence in string");
                char esc = s[idx++];
                switch (esc) {
                    case '"': result += '"'; break;
                    case '\\': result += '\\'; break;
                    case '/': result += '/'; break;
                    case 'b': result += '\b'; break;
                    case 'f': result += '\f'; break;
                    case 'n': result += '\n'; break;
                    case 'r': result += '\r'; break;
                    case 't': result += '\t'; break;
                    default: result += esc; break;
                }
            } else {
                result += c;
            }
        }
        throw JsonParseException("Unterminated string");
    }

    static JsonValue parseString(std::string_view s, size_t& idx) {
        return JsonValue(parseRawString(s, idx));
    }

    static JsonValue parseBool(std::string_view s, size_t& idx) {
        if (s.substr(idx, 4) == "true") {
            idx += 4;
            return JsonValue(true);
        }
        if (s.substr(idx, 5) == "false") {
            idx += 5;
            return JsonValue(false);
        }
        throw JsonParseException("Invalid boolean token at position " + std::to_string(idx));
    }

    static JsonValue parseNull(std::string_view s, size_t& idx) {
        if (s.substr(idx, 4) == "null") {
            idx += 4;
            return JsonValue(nullptr);
        }
        throw JsonParseException("Invalid null token at position " + std::to_string(idx));
    }

    static JsonValue parseNumber(std::string_view s, size_t& idx) {
        size_t start = idx;
        if (s[idx] == '-') idx++;
        while (idx < s.size() && (std::isdigit(static_cast<unsigned char>(s[idx])) || s[idx] == '.' || s[idx] == 'e' || s[idx] == 'E' || s[idx] == '+' || s[idx] == '-')) {
            idx++;
        }
        std::string numStr(s.substr(start, idx - start));
        try {
            double val = std::stod(numStr);
            return JsonValue(val);
        } catch (...) {
            throw JsonParseException("Malformed numeric string: " + numStr);
        }
    }
};

} // namespace aerofleet

