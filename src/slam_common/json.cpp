#include "slam_common/json.hpp"

#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace slam_common {
namespace {

const JsonValue kNullValue;

void append_utf8(std::string& out, uint32_t cp) {
    if (cp <= 0x7Fu) {
        out.push_back(static_cast<char>(cp));
    } else if (cp <= 0x7FFu) {
        out.push_back(static_cast<char>(0xC0u | ((cp >> 6) & 0x1Fu)));
        out.push_back(static_cast<char>(0x80u | (cp & 0x3Fu)));
    } else if (cp <= 0xFFFFu) {
        out.push_back(static_cast<char>(0xE0u | ((cp >> 12) & 0x0Fu)));
        out.push_back(static_cast<char>(0x80u | ((cp >> 6) & 0x3Fu)));
        out.push_back(static_cast<char>(0x80u | (cp & 0x3Fu)));
    } else {
        out.push_back(static_cast<char>(0xF0u | ((cp >> 18) & 0x07u)));
        out.push_back(static_cast<char>(0x80u | ((cp >> 12) & 0x3Fu)));
        out.push_back(static_cast<char>(0x80u | ((cp >> 6) & 0x3Fu)));
        out.push_back(static_cast<char>(0x80u | (cp & 0x3Fu)));
    }
}

class JsonParser {
public:
    explicit JsonParser(const std::string& s) : s_(s) {
        if (s_.size() >= 3 && static_cast<unsigned char>(s_[0]) == 0xEF &&
            static_cast<unsigned char>(s_[1]) == 0xBB && static_cast<unsigned char>(s_[2]) == 0xBF) {
            pos_ = 3;
        }
    }

    JsonValue parse_root() {
        skip_ws();
        JsonValue v = parse_value();
        skip_ws();
        if (pos_ != s_.size()) throw std::invalid_argument("Unexpected trailing characters in JSON");
        return v;
    }

private:
    void skip_ws() {
        while (pos_ < s_.size() && (s_[pos_] == ' ' || s_[pos_] == '\t' || s_[pos_] == '\r' || s_[pos_] == '\n')) {
            ++pos_;
        }
    }

    char peek() const {
        if (pos_ >= s_.size()) throw std::invalid_argument("Unexpected end of JSON");
        return s_[pos_];
    }

    char get() {
        char c = peek();
        ++pos_;
        return c;
    }

    JsonValue parse_value() {
        skip_ws();
        char c = peek();
        if (c == 'n') return parse_literal("null", JsonValue(nullptr));
        if (c == 't') return parse_literal("true", JsonValue(true));
        if (c == 'f') return parse_literal("false", JsonValue(false));
        if (c == '"') return JsonValue(parse_string());
        if (c == '[') return parse_array();
        if (c == '{') return parse_object();
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parse_number();
        throw std::invalid_argument("Invalid JSON token");
    }

    JsonValue parse_literal(const char* lit, JsonValue val) {
        for (const char* p = lit; *p; ++p) {
            if (get() != *p) throw std::invalid_argument(std::string("Expected ") + lit);
        }
        return val;
    }

    uint32_t parse_hex4() {
        if (pos_ + 4 > s_.size()) throw std::invalid_argument("Incomplete \\u escape");
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            char c = s_[pos_++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<uint32_t>(c - 'A' + 10);
            else throw std::invalid_argument("Invalid hex digit in \\u escape");
        }
        return v;
    }

    std::string parse_string() {
        if (get() != '"') throw std::invalid_argument("Expected '\"'");
        std::string out;
        while (pos_ < s_.size()) {
            char c = get();
            if (c == '"') return out;
            if (c == '\\') {
                char e = get();
                switch (e) {
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    case 'u': {
                        uint32_t cp = parse_hex4();
                        if (cp >= 0xD800u && cp <= 0xDBFFu && pos_ + 2 <= s_.size() && s_[pos_] == '\\' && s_[pos_ + 1] == 'u') {
                            pos_ += 2;
                            uint32_t low = parse_hex4();
                            if (low >= 0xDC00u && low <= 0xDFFFu) {
                                cp = 0x10000u + (((cp - 0xD800u) << 10) | (low - 0xDC00u));
                            }
                        }
                        append_utf8(out, cp);
                        break;
                    }
                    default: throw std::invalid_argument("Invalid JSON escape");
                }
            } else {
                out.push_back(c);
            }
        }
        throw std::invalid_argument("Unterminated JSON string");
    }

    JsonValue parse_number() {
        size_t start = pos_;
        bool is_float = false;
        if (s_[pos_] == '-') ++pos_;
        if (pos_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[pos_]))) {
            throw std::invalid_argument("Invalid JSON number");
        }
        while (pos_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[pos_]))) ++pos_;
        if (pos_ < s_.size() && s_[pos_] == '.') {
            is_float = true;
            ++pos_;
            if (pos_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[pos_]))) {
                throw std::invalid_argument("Invalid JSON fractional number");
            }
            while (pos_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[pos_]))) ++pos_;
        }
        if (pos_ < s_.size() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
            is_float = true;
            ++pos_;
            if (pos_ < s_.size() && (s_[pos_] == '+' || s_[pos_] == '-')) ++pos_;
            if (pos_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[pos_]))) {
                throw std::invalid_argument("Invalid JSON exponent");
            }
            while (pos_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[pos_]))) ++pos_;
        }
        std::string token = s_.substr(start, pos_ - start);
        if (!is_float) {
            try {
                size_t idx = 0;
                long long v = std::stoll(token, &idx, 10);
                if (idx == token.size()) return JsonValue(static_cast<int64_t>(v));
            } catch (...) {
            }
        }
        double d = std::stod(token);
        if (!std::isfinite(d)) throw std::invalid_argument("Non-finite JSON number");
        return JsonValue(d);
    }

    JsonValue parse_array() {
        get();
        JsonValue::Array arr;
        skip_ws();
        if (peek() == ']') {
            get();
            return JsonValue(std::move(arr));
        }
        while (true) {
            arr.push_back(parse_value());
            skip_ws();
            char c = get();
            if (c == ']') break;
            if (c != ',') throw std::invalid_argument("Expected ',' or ']' in JSON array");
        }
        return JsonValue(std::move(arr));
    }

    JsonValue parse_object() {
        get();
        JsonValue::Object obj;
        skip_ws();
        if (peek() == '}') {
            get();
            return JsonValue(std::move(obj));
        }
        while (true) {
            skip_ws();
            std::string key = parse_string();
            skip_ws();
            if (get() != ':') throw std::invalid_argument("Expected ':' in JSON object");
            obj[key] = parse_value();
            skip_ws();
            char c = get();
            if (c == '}') break;
            if (c != ',') throw std::invalid_argument("Expected ',' or '}' in JSON object");
        }
        return JsonValue(std::move(obj));
    }

    const std::string& s_;
    size_t pos_ = 0;
};

void escape_json_string(std::ostringstream& os, const std::string& s) {
    os << '"';
    for (unsigned char c : s) {
        switch (c) {
            case '"': os << "\\\""; break;
            case '\\': os << "\\\\"; break;
            case '\b': os << "\\b"; break;
            case '\f': os << "\\f"; break;
            case '\n': os << "\\n"; break;
            case '\r': os << "\\r"; break;
            case '\t': os << "\\t"; break;
            default:
                if (c < 0x20u) {
                    os << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
                } else {
                    os << static_cast<char>(c);
                }
        }
    }
    os << '"';
}

void dump_impl(std::ostringstream& os, const JsonValue& v, int indent, int depth) {
    switch (v.type()) {
        case JsonValue::Type::Null:
            os << "null";
            break;
        case JsonValue::Type::Bool:
            os << (v.as_bool() ? "true" : "false");
            break;
        case JsonValue::Type::Int:
            os << v.as_int();
            break;
        case JsonValue::Type::Double: {
            double d = v.as_double();
            if (!std::isfinite(d)) throw std::invalid_argument("Out of range float values are not JSON compliant");
            std::ostringstream num;
            num << std::setprecision(15) << d;
            std::string s = num.str();
            if (s.find('.') == std::string::npos && s.find('e') == std::string::npos && s.find('E') == std::string::npos) {
                s += ".0";
            }
            os << s;
            break;
        }
        case JsonValue::Type::String:
            escape_json_string(os, v.as_string());
            break;
        case JsonValue::Type::Array: {
            const auto& arr = v.as_array();
            if (arr.empty()) {
                os << "[]";
                break;
            }
            os << '[';
            if (indent >= 0) os << '\n';
            for (size_t i = 0; i < arr.size(); ++i) {
                if (indent >= 0) os << std::string(static_cast<size_t>((depth + 1) * indent), ' ');
                dump_impl(os, arr[i], indent, depth + 1);
                if (i + 1 < arr.size()) os << ',';
                if (indent >= 0) os << '\n';
            }
            if (indent >= 0) os << std::string(static_cast<size_t>(depth * indent), ' ');
            os << ']';
            break;
        }
        case JsonValue::Type::Object: {
            const auto& obj = v.as_object();
            if (obj.empty()) {
                os << "{}";
                break;
            }
            os << '{';
            if (indent >= 0) os << '\n';
            size_t idx = 0;
            for (const auto& kv : obj) {
                if (indent >= 0) os << std::string(static_cast<size_t>((depth + 1) * indent), ' ');
                escape_json_string(os, kv.first);
                os << (indent >= 0 ? ": " : ":");
                dump_impl(os, kv.second, indent, depth + 1);
                if (++idx < obj.size()) os << ',';
                if (indent >= 0) os << '\n';
            }
            if (indent >= 0) os << std::string(static_cast<size_t>(depth * indent), ' ');
            os << '}';
            break;
        }
    }
}

std::string random_hex_8() {
    static thread_local std::mt19937_64 rng(std::random_device{}());
    uint32_t val = static_cast<uint32_t>(rng() & 0xffffffffu);
    std::ostringstream os;
    os << std::hex << std::setw(8) << std::setfill('0') << val;
    return os.str();
}

}  // namespace

bool JsonValue::as_bool(bool def) const {
    return type_ == Type::Bool ? bool_val_ : def;
}

int64_t JsonValue::as_int(int64_t def) const {
    if (type_ == Type::Int) return int_val_;
    if (type_ == Type::Double) return static_cast<int64_t>(double_val_);
    return def;
}

double JsonValue::as_double(double def) const {
    if (type_ == Type::Double) return double_val_;
    if (type_ == Type::Int) return static_cast<double>(int_val_);
    return def;
}

const std::string& JsonValue::as_string() const {
    if (type_ != Type::String) throw std::runtime_error("JSON value is not a string");
    return str_val_;
}

const JsonValue::Array& JsonValue::as_array() const {
    if (type_ != Type::Array) throw std::runtime_error("JSON value is not an array");
    return arr_val_;
}

JsonValue::Array& JsonValue::as_array() {
    if (type_ != Type::Array) throw std::runtime_error("JSON value is not an array");
    return arr_val_;
}

const JsonValue::Object& JsonValue::as_object() const {
    if (type_ != Type::Object) throw std::runtime_error("JSON value is not an object");
    return obj_val_;
}

JsonValue::Object& JsonValue::as_object() {
    if (type_ != Type::Object) throw std::runtime_error("JSON value is not an object");
    return obj_val_;
}

bool JsonValue::contains(const std::string& key) const {
    return type_ == Type::Object && obj_val_.find(key) != obj_val_.end();
}

const JsonValue& JsonValue::get(const std::string& key) const {
    if (type_ != Type::Object) return kNullValue;
    auto it = obj_val_.find(key);
    return it == obj_val_.end() ? kNullValue : it->second;
}

JsonValue& JsonValue::operator[](const std::string& key) {
    if (type_ == Type::Null) type_ = Type::Object;
    if (type_ != Type::Object) throw std::runtime_error("JSON value is not an object");
    return obj_val_[key];
}

void JsonValue::push_back(JsonValue v) {
    if (type_ == Type::Null) type_ = Type::Array;
    if (type_ != Type::Array) throw std::runtime_error("JSON value is not an array");
    arr_val_.push_back(std::move(v));
}

std::string JsonValue::dump(int indent) const {
    std::ostringstream os;
    dump_impl(os, *this, indent, 0);
    return os.str();
}

JsonValue JsonValue::parse(const std::string& text) {
    JsonParser parser(text);
    return parser.parse_root();
}

std::string read_text_utf8(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file: " + path.string());
    std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (raw.size() >= 3 && static_cast<unsigned char>(raw[0]) == 0xEF &&
        static_cast<unsigned char>(raw[1]) == 0xBB && static_cast<unsigned char>(raw[2]) == 0xBF) {
        raw.erase(0, 3);
    }
    return raw;
}

JsonValue read_json_file(const std::filesystem::path& path) {
    return JsonValue::parse(read_text_utf8(path));
}

JsonValue read_json_safe(const std::filesystem::path& path) {
    try {
        if (!std::filesystem::is_regular_file(path)) return JsonValue::object();
        return read_json_file(path);
    } catch (...) {
        return JsonValue::object();
    }
}

void save_json(const std::filesystem::path& path, const JsonValue& value, bool trailing_newline) {
    std::string payload = value.dump(2);
    if (trailing_newline) payload.push_back('\n');
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());

    std::filesystem::path tmp = path;
    tmp += "." + random_hex_8() + ".tmp";
    auto cleanup = [&]() {
        std::error_code ec;
        std::filesystem::remove(tmp, ec);
    };
    try {
        {
            std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
            if (!out) throw std::runtime_error("Failed to create temp JSON file: " + tmp.string());
            out.write(payload.data(), static_cast<std::streamsize>(payload.size()));
            if (!out) throw std::runtime_error("Failed to write temp JSON file: " + tmp.string());
        }
        bool replaced = false;
        for (int attempt = 0; attempt < 20; ++attempt) {
            std::error_code ec;
            std::filesystem::rename(tmp, path, ec);
            if (!ec) {
                replaced = true;
                break;
            }
            if (attempt < 19) std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        if (replaced) {
            cleanup();
            return;
        }
        for (int attempt = 0; attempt < 10; ++attempt) {
            std::ofstream direct(path, std::ios::binary | std::ios::trunc);
            if (direct) {
                direct.write(payload.data(), static_cast<std::streamsize>(payload.size()));
                if (direct) {
                    cleanup();
                    return;
                }
            }
            if (attempt == 9) {
                cleanup();
                throw std::runtime_error("Failed to write JSON file: " + path.string());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    } catch (...) {
        cleanup();
        throw;
    }
}

std::string html_escape(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char c : text) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#x27;"; break;
            default: out.push_back(c); break;
        }
    }
    return out;
}

}  // namespace slam_common
