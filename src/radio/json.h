#pragma once
// A small, strict JSON reader and the writer helpers the radio routes need.
//
// The rest of the control plane gets by with strstr over flat bodies, but a radio spec is
// nested (carriers → towers), carries strings and nulls, and has to be validated field by
// field with a useful error — a flat scan cannot tell a tower's "lat" from a hole's. This
// stays dependency-free like the rest of src/api: RFC 8259 grammar, no extensions.
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace skysim::radio::json {

struct Value {
    enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

    Type type = Type::kNull;
    bool boolean = false;
    double number = 0.0;
    // kString: the decoded text. kNumber: the literal as written, so an integer too large
    // for a double (a 64-bit seed) can still be read exactly.
    std::string text;
    // kArray: the elements. kObject: the member values, parallel to `keys`.
    std::vector<Value> items;
    std::vector<std::string> keys;

    bool is_null() const { return type == Type::kNull; }
    bool is_number() const { return type == Type::kNumber; }
    bool is_string() const { return type == Type::kString; }
    bool is_array() const { return type == Type::kArray; }
    bool is_object() const { return type == Type::kObject; }

    // Member lookup on an object; null when absent or when this is not an object.
    const Value *find(std::string_view key) const;
};

// Deepest nesting accepted. A radio spec is four levels deep; this only exists so a
// hostile body cannot recurse the parser off the end of the stack.
inline constexpr int kMaxDepth = 32;

// Parses one complete JSON document (surrounding whitespace allowed). On failure returns
// nullopt and sets `error` to a message naming the byte offset. Duplicate object keys are
// rejected: two values for one field is ambiguous, and a spec must mean one thing.
std::optional<Value> parse(std::string_view text, std::string &error);

// Appends `s` as a quoted JSON string with the mandatory escapes.
void append_string(std::string &out, std::string_view s);

// Appends a finite double in its shortest round-trip form ("47", "8.015", "-1e-07").
void append_number(std::string &out, double v);

// Appends a fixed-point number with `decimals` places, or null when `v` is absent.
void append_fixed(std::string &out, std::optional<double> v, int decimals);

} // namespace skysim::radio::json
