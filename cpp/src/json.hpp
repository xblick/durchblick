// Internal: the little JSON Durchblick needs (attachment and info record, protocol.hpp): flat objects whose values
// are strings, integers, booleans, null or arrays of strings. Anything else is refused, not guessed.
#pragma once

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace xblick::json
{
    using Value = std::variant<std::nullptr_t, bool, std::int64_t, std::string, std::vector<std::string>>;
    using Object = std::map<std::string, Value>;

    class Error : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    inline std::string quote(const std::string& s)
    {
        std::string out = "\"";
        for (char c : s)
        {
            switch (c)
            {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20)
                    {
                        char buf[8];
                        std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                        out += buf;
                    }
                    else
                    {
                        out += c;
                    }
            }
        }
        return out + "\"";
    }

    // Writes the members in the given order (compact, as Python's json.dumps(separators=(",", ":"))).
    inline std::string write(const std::vector<std::pair<std::string, Value>>& members)
    {
        std::string out = "{";
        for (std::size_t i = 0; i < members.size(); ++i)
        {
            if (i)
            {
                out += ",";
            }
            out += quote(members[i].first) + ":";
            const Value& v = members[i].second;
            if (std::holds_alternative<std::nullptr_t>(v))
            {
                out += "null";
            }
            else if (const bool* b = std::get_if<bool>(&v))
            {
                out += *b ? "true" : "false";
            }
            else if (const std::int64_t* n = std::get_if<std::int64_t>(&v))
            {
                out += std::to_string(*n);
            }
            else if (const std::string* s = std::get_if<std::string>(&v))
            {
                out += quote(*s);
            }
            else
            {
                const auto& list = std::get<std::vector<std::string>>(v);
                out += "[";
                for (std::size_t j = 0; j < list.size(); ++j)
                {
                    out += (j ? "," : "") + quote(list[j]);
                }
                out += "]";
            }
        }
        return out + "}";
    }

    class Parser
    {
    public:
        explicit Parser(const std::string& text) : s_(text) {}

        Object object()
        {
            Object out;
            expect('{');
            skip();
            if (peek() == '}')
            {
                ++i_;
                return out;
            }
            while (true)
            {
                skip();
                std::string key = string();
                skip();
                expect(':');
                skip();
                out[key] = value();
                skip();
                if (peek() == ',')
                {
                    ++i_;
                    continue;
                }
                expect('}');
                return out;
            }
        }

    private:
        char peek() const
        {
            if (i_ >= s_.size())
            {
                throw Error("unexpected end of JSON");
            }
            return s_[i_];
        }
        void skip()
        {
            while (i_ < s_.size() and std::isspace(static_cast<unsigned char>(s_[i_])))
            {
                ++i_;
            }
        }
        void expect(char c)
        {
            skip();
            if (peek() != c)
            {
                throw Error(std::string("JSON: expected '") + c + "' at " + std::to_string(i_));
            }
            ++i_;
        }
        bool literal(const char* word)
        {
            const std::size_t n = std::strlen(word);
            if (s_.compare(i_, n, word) == 0)
            {
                i_ += n;
                return true;
            }
            return false;
        }
        std::string string()
        {
            expect('"');
            std::string out;
            while (peek() != '"')
            {
                char c = s_[i_++];
                if (c == '\\')
                {
                    char e = peek();
                    ++i_;
                    switch (e)
                    {
                        case '"': out += '"'; break;
                        case '\\': out += '\\'; break;
                        case '/': out += '/'; break;
                        case 'n': out += '\n'; break;
                        case 'r': out += '\r'; break;
                        case 't': out += '\t'; break;
                        case 'b': out += '\b'; break;
                        case 'f': out += '\f'; break;
                        case 'u':
                        {
                            const unsigned code = std::stoul(s_.substr(i_, 4), nullptr, 16);
                            i_ += 4;
                            if (code < 0x80)
                            {
                                out += static_cast<char>(code);
                            }
                            else if (code < 0x800)
                            {
                                out += static_cast<char>(0xC0 | (code >> 6));
                                out += static_cast<char>(0x80 | (code & 0x3F));
                            }
                            else
                            {
                                out += static_cast<char>(0xE0 | (code >> 12));
                                out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                                out += static_cast<char>(0x80 | (code & 0x3F));
                            }
                            break;
                        }
                        default: throw Error("JSON: bad escape");
                    }
                }
                else
                {
                    out += c;
                }
            }
            ++i_;
            return out;
        }
        Value value()
        {
            const char c = peek();
            if (c == '"')
            {
                return string();
            }
            if (c == '[')
            {
                ++i_;
                std::vector<std::string> list;
                skip();
                if (peek() == ']')
                {
                    ++i_;
                    return list;
                }
                while (true)
                {
                    skip();
                    list.push_back(string());
                    skip();
                    if (peek() == ',')
                    {
                        ++i_;
                        continue;
                    }
                    expect(']');
                    return list;
                }
            }
            if (literal("null"))
            {
                return nullptr;
            }
            if (literal("true"))
            {
                return true;
            }
            if (literal("false"))
            {
                return false;
            }
            std::size_t used = 0;
            const std::int64_t n = std::stoll(s_.substr(i_), &used);
            if (i_ + used < s_.size() and (s_[i_ + used] == '.' or s_[i_ + used] == 'e' or s_[i_ + used] == 'E'))
            {
                throw Error("JSON: only integers are expected");
            }
            i_ += used;
            return n;
        }

        const std::string& s_;
        std::size_t i_ = 0;
    };

    inline Object parse(const std::string& text)
    {
        return Parser(text).object();
    }

    template <typename T>
    std::optional<T> get(const Object& o, const std::string& key)
    {
        auto it = o.find(key);
        if (it == o.end())
        {
            return std::nullopt;
        }
        if (const T* v = std::get_if<T>(&it->second))
        {
            return *v;
        }
        return std::nullopt;
    }
}  // namespace xblick::json
