// See json.h. Recursive descent over a string_view; numbers go through std::from_chars so
// the result never depends on the process locale.
#include "radio/json.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_set>

namespace skysim::radio::json {

const Value *Value::find(std::string_view key) const {
    if (type != Type::kObject) {
        return nullptr;
    }
    for (size_t i = 0; i < keys.size(); ++i) {
        if (keys[i] == key) {
            return &items[i];
        }
    }
    return nullptr;
}

namespace {

// Length of the well-formed UTF-8 sequence starting at s[0] (n bytes available), or 0 when
// it is not one: a stray continuation byte, an overlong form, a surrogate, past U+10FFFF, or
// cut short. Names a spec carries are echoed in every response, so they must be text.
size_t utf8_length(const unsigned char *s, size_t n) {
    const unsigned char c = s[0];
    if (c < 0x80) {
        return 1;
    }
    size_t len = 0;
    uint32_t cp = 0;
    if (c >= 0xC2 && c <= 0xDF) {
        len = 2;
        cp = c & 0x1Fu;
    } else if (c >= 0xE0 && c <= 0xEF) {
        len = 3;
        cp = c & 0x0Fu;
    } else if (c >= 0xF0 && c <= 0xF4) {
        len = 4;
        cp = c & 0x07u;
    } else {
        return 0;
    }
    if (n < len) {
        return 0;
    }
    for (size_t i = 1; i < len; ++i) {
        if ((s[i] & 0xC0u) != 0x80u) {
            return 0;
        }
        cp = (cp << 6) | (s[i] & 0x3Fu);
    }
    const bool overlong = (len == 3 && cp < 0x800) || (len == 4 && cp < 0x10000);
    const bool surrogate = cp >= 0xD800 && cp <= 0xDFFF;
    return overlong || surrogate || cp > 0x10FFFF ? 0 : len;
}

class Parser {
  public:
    Parser(std::string_view text, std::string &error) : text_(text), error_(error) {}

    std::optional<Value> document() {
        Value v;
        skip_space();
        if (!value(v, 0)) {
            return std::nullopt;
        }
        skip_space();
        if (pos_ != text_.size()) {
            return fail("unexpected trailing characters");
        }
        return v;
    }

  private:
    std::nullopt_t fail(const char *what) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "invalid JSON at offset %zu: %s", pos_, what);
        error_ = buf;
        return std::nullopt;
    }

    bool at_end() const { return pos_ >= text_.size(); }
    char peek() const { return at_end() ? '\0' : text_[pos_]; }

    void skip_space() {
        while (!at_end() && (peek() == ' ' || peek() == '\t' || peek() == '\n' || peek() == '\r')) {
            ++pos_;
        }
    }

    bool literal(std::string_view word) {
        if (text_.substr(pos_, word.size()) != word) {
            fail("unknown literal");
            return false;
        }
        pos_ += word.size();
        return true;
    }

    bool value(Value &out, int depth) {
        if (depth > kMaxDepth) {
            fail("nesting too deep");
            return false;
        }
        switch (peek()) {
        case '{':
            return object(out, depth);
        case '[':
            return array(out, depth);
        case '"':
            out.type = Value::Type::kString;
            return string(out.text);
        case 't':
            out.type = Value::Type::kBool;
            out.boolean = true;
            return literal("true");
        case 'f':
            out.type = Value::Type::kBool;
            return literal("false");
        case 'n':
            out.type = Value::Type::kNull;
            return literal("null");
        default:
            return number(out);
        }
    }

    bool object(Value &out, int depth) {
        out.type = Value::Type::kObject;
        ++pos_; // '{'
        skip_space();
        if (peek() == '}') {
            ++pos_;
            return true;
        }
        // A set, not a scan of the keys so far: a large body of distinct keys must not cost
        // a quadratic number of comparisons on a control-plane thread.
        std::unordered_set<std::string> seen;
        while (true) {
            skip_space();
            if (peek() != '"') {
                fail("expected a quoted key");
                return false;
            }
            std::string key;
            if (!string(key)) {
                return false;
            }
            if (!seen.insert(key).second) {
                fail("duplicate key");
                return false;
            }
            skip_space();
            if (peek() != ':') {
                fail("expected ':'");
                return false;
            }
            ++pos_;
            skip_space();
            Value member;
            if (!value(member, depth + 1)) {
                return false;
            }
            out.keys.push_back(std::move(key));
            out.items.push_back(std::move(member));
            skip_space();
            if (peek() == ',') {
                ++pos_;
                continue;
            }
            if (peek() == '}') {
                ++pos_;
                return true;
            }
            fail("expected ',' or '}'");
            return false;
        }
    }

    bool array(Value &out, int depth) {
        out.type = Value::Type::kArray;
        ++pos_; // '['
        skip_space();
        if (peek() == ']') {
            ++pos_;
            return true;
        }
        while (true) {
            skip_space();
            Value item;
            if (!value(item, depth + 1)) {
                return false;
            }
            out.items.push_back(std::move(item));
            skip_space();
            if (peek() == ',') {
                ++pos_;
                continue;
            }
            if (peek() == ']') {
                ++pos_;
                return true;
            }
            fail("expected ',' or ']'");
            return false;
        }
    }

    static int hex_digit(char c) {
        if (c >= '0' && c <= '9') {
            return c - '0';
        }
        if (c >= 'a' && c <= 'f') {
            return c - 'a' + 10;
        }
        if (c >= 'A' && c <= 'F') {
            return c - 'A' + 10;
        }
        return -1;
    }

    // Four hex digits after "\u"; -1 when malformed.
    long hex4() {
        if (pos_ + 4 > text_.size()) {
            return -1;
        }
        long v = 0;
        for (int i = 0; i < 4; ++i) {
            const int d = hex_digit(text_[pos_ + static_cast<size_t>(i)]);
            if (d < 0) {
                return -1;
            }
            v = v * 16 + d;
        }
        pos_ += 4;
        return v;
    }

    static void append_utf8(std::string &out, uint32_t cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    bool unicode_escape(std::string &out) {
        const long hi = hex4();
        if (hi < 0) {
            fail("bad \\u escape");
            return false;
        }
        uint32_t cp = static_cast<uint32_t>(hi);
        if (hi >= 0xDC00 && hi <= 0xDFFF) {
            fail("unpaired surrogate");
            return false;
        }
        if (hi >= 0xD800 && hi <= 0xDBFF) {
            // A high surrogate must be followed by an escaped low one.
            if (text_.substr(pos_, 2) != "\\u") {
                fail("unpaired surrogate");
                return false;
            }
            pos_ += 2;
            const long lo = hex4();
            if (lo < 0xDC00 || lo > 0xDFFF) {
                fail("unpaired surrogate");
                return false;
            }
            cp =
                0x10000 + ((static_cast<uint32_t>(hi) - 0xD800) << 10) + (static_cast<uint32_t>(lo) - 0xDC00);
        }
        append_utf8(out, cp);
        return true;
    }

    bool string(std::string &out) {
        ++pos_; // opening quote
        while (true) {
            if (at_end()) {
                fail("unterminated string");
                return false;
            }
            const char c = text_[pos_++];
            if (c == '"') {
                return true;
            }
            if (static_cast<unsigned char>(c) < 0x20) {
                fail("control character in string");
                return false;
            }
            if (static_cast<unsigned char>(c) >= 0x80) {
                const size_t len =
                    utf8_length(reinterpret_cast<const unsigned char *>(text_.data()) + pos_ - 1,
                                text_.size() - pos_ + 1);
                if (len == 0) {
                    --pos_;
                    fail("invalid UTF-8 in string");
                    return false;
                }
                out.append(text_.substr(pos_ - 1, len));
                pos_ += len - 1;
                continue;
            }
            if (c != '\\') {
                out += c;
                continue;
            }
            const char e = peek();
            ++pos_;
            switch (e) {
            case '"':
            case '\\':
            case '/':
                out += e;
                break;
            case 'b':
                out += '\b';
                break;
            case 'f':
                out += '\f';
                break;
            case 'n':
                out += '\n';
                break;
            case 'r':
                out += '\r';
                break;
            case 't':
                out += '\t';
                break;
            case 'u':
                if (!unicode_escape(out)) {
                    return false;
                }
                break;
            default:
                --pos_;
                fail("bad escape");
                return false;
            }
        }
    }

    static bool is_digit(char c) { return c >= '0' && c <= '9'; }

    bool digits() {
        if (!is_digit(peek())) {
            return false;
        }
        while (is_digit(peek())) {
            ++pos_;
        }
        return true;
    }

    // -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?
    bool number(Value &out) {
        const size_t start = pos_;
        if (peek() == '-') {
            ++pos_;
        }
        if (peek() == '0') {
            ++pos_;
        } else if (!digits()) {
            pos_ = start;
            fail("unexpected character");
            return false;
        }
        if (peek() == '.') {
            ++pos_;
            if (!digits()) {
                fail("digit expected after '.'");
                return false;
            }
        }
        if (peek() == 'e' || peek() == 'E') {
            ++pos_;
            if (peek() == '+' || peek() == '-') {
                ++pos_;
            }
            if (!digits()) {
                fail("digit expected in exponent");
                return false;
            }
        }
        out.type = Value::Type::kNumber;
        out.text.assign(text_.substr(start, pos_ - start));
        // Overflow (1e999) and underflow (1e-999) both land here: neither is a value any
        // field of a radio spec can mean, and JSON has no other way to spell a non-finite.
        const auto result = std::from_chars(out.text.data(), out.text.data() + out.text.size(), out.number);
        if (result.ec != std::errc()) {
            pos_ = start;
            fail("number out of range");
            return false;
        }
        return true;
    }

    std::string_view text_;
    std::string &error_;
    size_t pos_ = 0;
};

} // namespace

std::optional<Value> parse(std::string_view text, std::string &error) {
    return Parser(text, error).document();
}

void append_string(std::string &out, std::string_view s) {
    out += '"';
    for (size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (static_cast<unsigned char>(c) >= 0x80) {
            // Text that is not UTF-8 (a query parameter echoed in an error, say) is replaced
            // byte by byte, so the response stays valid JSON whatever it quotes.
            const size_t len =
                utf8_length(reinterpret_cast<const unsigned char *>(s.data()) + i, s.size() - i);
            if (len == 0) {
                out += "\\ufffd";
            } else {
                out.append(s.substr(i, len));
                i += len - 1;
            }
            continue;
        }
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x",
                              static_cast<unsigned>(static_cast<unsigned char>(c)));
                out += buf;
            } else {
                out += c;
            }
        }
    }
    out += '"';
}

void append_number(std::string &out, double v) {
    // 32 bytes holds any double's shortest form (at most 24 characters), so this cannot fail.
    char buf[32];
    const auto result = std::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, static_cast<size_t>(result.ptr - buf));
}

void append_fixed(std::string &out, std::optional<double> v, int decimals) {
    // JSON has no spelling for NaN or infinity: a value that is not a number is not reported.
    if (!v.has_value() || !std::isfinite(*v)) {
        out += "null";
        return;
    }
    // Room for any finite double at the precisions used here (DBL_MAX is 309 digits).
    char buf[352];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, *v);
    out += buf;
}

} // namespace skysim::radio::json
