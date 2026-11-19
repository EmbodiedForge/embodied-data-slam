#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace slam_common {

/// Lightweight JSON value supporting Null, Bool, Int64, Double, String, Array, and Object.
class JsonValue {
public:
    enum class Type { Null, Bool, Int, Double, String, Array, Object };
    using Array = std::vector<JsonValue>;
    using Object = std::map<std::string, JsonValue>;

    JsonValue() : type_(Type::Null) {}
    JsonValue(std::nullptr_t) : type_(Type::Null) {}
    JsonValue(bool b) : type_(Type::Bool), bool_val_(b) {}
    JsonValue(int i) : type_(Type::Int), int_val_(i) {}
    JsonValue(int64_t i) : type_(Type::Int), int_val_(i) {}
    JsonValue(size_t i) : type_(Type::Int), int_val_(static_cast<int64_t>(i)) {}
    JsonValue(double d) : type_(Type::Double), double_val_(d) {}
    JsonValue(const char* s) : type_(Type::String), str_val_(s ? s : "") {}
    JsonValue(std::string s) : type_(Type::String), str_val_(std::move(s)) {}
    JsonValue(Array arr) : type_(Type::Array), arr_val_(std::move(arr)) {}
    JsonValue(Object obj) : type_(Type::Object), obj_val_(std::move(obj)) {}

    static JsonValue array() { return JsonValue(Array{}); }
    static JsonValue object() { return JsonValue(Object{}); }

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::Null; }
    bool is_bool() const { return type_ == Type::Bool; }
    bool is_int() const { return type_ == Type::Int; }
    bool is_double() const { return type_ == Type::Double; }
    bool is_number() const { return type_ == Type::Int || type_ == Type::Double; }
    bool is_string() const { return type_ == Type::String; }
    bool is_array() const { return type_ == Type::Array; }
    bool is_object() const { return type_ == Type::Object; }

    bool as_bool(bool def = false) const;
    int64_t as_int(int64_t def = 0) const;
    double as_double(double def = 0.0) const;
    const std::string& as_string() const;
    const Array& as_array() const;
    Array& as_array();
    const Object& as_object() const;
    Object& as_object();

    bool contains(const std::string& key) const;
    const JsonValue& get(const std::string& key) const;
    JsonValue& operator[](const std::string& key);
    void push_back(JsonValue v);

    std::string dump(int indent = 2) const;
    static JsonValue parse(const std::string& text);

private:
    Type type_ = Type::Null;
    bool bool_val_ = false;
    int64_t int_val_ = 0;
    double double_val_ = 0.0;
    std::string str_val_;
    Array arr_val_;
    Object obj_val_;
};

/// Read UTF-8 (with optional BOM) text from file.
std::string read_text_utf8(const std::filesystem::path& path);

/// Parse JSON from file; throws on missing/invalid JSON.
JsonValue read_json_file(const std::filesystem::path& path);

/// Parse JSON from file; returns empty object on error.
JsonValue read_json_safe(const std::filesystem::path& path);

/// Atomically serialize and save JSON with retry/fallback for transient Windows locks.
void save_json(const std::filesystem::path& path, const JsonValue& value, bool trailing_newline = false);

/// HTML-escape special characters for offline report generation.
std::string html_escape(const std::string& text);

}  // namespace slam_common
