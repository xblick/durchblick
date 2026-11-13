// CameraInfo from and to ROS's calibration file (msg.hpp), without a YAML library: the format is fixed
// (camera_calibration_parsers' parse_yml.cpp writes it, ROS's camera_calibration and ost.yaml use it), so this reads
// exactly it and refuses the rest. Python's msg.py reads and writes the same.
//
//   image_width: 1280
//   image_height: 720
//   camera_name: azurekinect
//   camera_matrix:
//     rows: 3
//     cols: 3
//     data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]      (a list may go on over several lines)
//   distortion_model: rational_polynomial
//   distortion_coefficients:
//     rows: 1
//     cols: 8
//     data: [k1, k2, p1, p2, k3, k4, k5, k6]
//   rectification_matrix: (rows 3, cols 3, data)
//   projection_matrix: (rows 3, cols 4, data)
#include <algorithm>
#include <charconv>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <system_error>

#include "xblick/msg.hpp"

namespace xblick
{
    namespace
    {
        struct Matrix
        {
            std::optional<long> rows, cols;
            std::optional<std::vector<double>> data;
        };

        [[noreturn]] void fail(int line, const std::string& what)
        {
            throw std::invalid_argument("calibration file, line " + std::to_string(line) + ": " + what);
        }

        std::string trim(const std::string& s)
        {
            const auto begin = s.find_first_not_of(" \t\r");
            if (begin == std::string::npos)
            {
                return "";
            }
            return s.substr(begin, s.find_last_not_of(" \t\r") - begin + 1);
        }

        // a scalar, without the quotes YAML allows around it
        std::string unquote(const std::string& s)
        {
            if (s.size() >= 2 and (s.front() == '"' or s.front() == '\'') and s.back() == s.front())
            {
                return s.substr(1, s.size() - 2);
            }
            return s;
        }

        double number(const std::string& text, int line)
        {
            const std::string s = trim(text);
            double value = 0.0;
            const auto [end, error] = std::from_chars(s.data(), s.data() + s.size(), value);
            if (s.empty() or error != std::errc() or end != s.data() + s.size())
            {
                fail(line, "'" + s + "' is not a number");
            }
            return value;
        }

        long integer(const std::string& text, int line)
        {
            const std::string s = trim(text);
            long value = 0;
            const auto [end, error] = std::from_chars(s.data(), s.data() + s.size(), value);
            if (s.empty() or error != std::errc() or end != s.data() + s.size() or value < 0)
            {
                fail(line, "'" + s + "' is not a count");
            }
            return value;
        }

        void check(const std::string& name, const Matrix& m, long rows, long cols, int line)
        {
            if (not m.rows or not m.cols or not m.data)
            {
                fail(line, name + " needs rows, cols and data");
            }
            if ((rows >= 0 and *m.rows != rows) or (cols >= 0 and *m.cols != cols) or
                static_cast<long>(m.data->size()) != *m.rows * *m.cols)
            {
                fail(line, name + ": " + std::to_string(*m.rows) + " x " + std::to_string(*m.cols) + " with " +
                               std::to_string(m.data->size()) + " values" +
                               (rows >= 0 ? ", " + std::to_string(rows) + " x " + std::to_string(cols) + " needed"
                                          : ""));
            }
        }

        void list(std::ostringstream& out, const std::string& name, long rows, long cols,
                  const std::vector<double>& data)
        {
            out << name << ":\n  rows: " << rows << "\n  cols: " << cols << "\n  data: [";
            for (std::size_t i = 0; i < data.size(); ++i)
            {
                char buffer[32];
                const auto end = std::to_chars(buffer, buffer + sizeof(buffer), data[i]).ptr;  // shortest exact
                out << (i ? ", " : "") << std::string(buffer, end);
            }
            out << "]\n";
        }
    }  // namespace

    CameraInfo CameraInfo::from_yaml(const std::string& text)
    {
        std::map<std::string, std::string> scalars;
        std::map<std::string, Matrix> matrices;
        std::string current;      // the matrix whose indented keys follow
        std::string pending;      // a list not closed yet
        int pending_line = 0;
        std::vector<double>* pending_data = nullptr;
        std::istringstream in(text);
        std::string raw;
        int line = 0;
        while (std::getline(in, raw))
        {
            ++line;
            const auto hash = raw.find('#');
            const std::string content = hash == std::string::npos ? raw : raw.substr(0, hash);
            if (pending_data)  // inside [ ... ]
            {
                if (not trim(content).empty() and content.front() != ' ' and content.front() != '\t')
                {
                    fail(pending_line, "list without ']'");  // the next key came first
                }
                pending += " " + content;
                if (pending.find(']') == std::string::npos)
                {
                    continue;
                }
            }
            else
            {
                if (trim(content).empty() or trim(content) == "---")
                {
                    continue;
                }
                const bool indented = content.front() == ' ' or content.front() == '\t';
                const auto colon = content.find(':');
                if (colon == std::string::npos)
                {
                    fail(line, "'key: value' expected");
                }
                const std::string key = trim(content.substr(0, colon));
                const std::string value = trim(content.substr(colon + 1));
                if (not indented)
                {
                    current.clear();
                    if (key == "image_width" or key == "image_height" or key == "camera_name" or
                        key == "distortion_model")
                    {
                        if (scalars.count(key))
                        {
                            fail(line, key + " twice");
                        }
                        scalars[key] = unquote(value);
                    }
                    else if (key == "camera_matrix" or key == "distortion_coefficients" or
                             key == "rectification_matrix" or key == "projection_matrix")
                    {
                        if (not value.empty() or matrices.count(key))
                        {
                            fail(line, key + ": rows, cols and data on the lines below, once");
                        }
                        current = key;
                        matrices[key];
                    }
                    else
                    {
                        fail(line, "unknown key '" + key + "'");
                    }
                    continue;
                }
                if (current.empty())
                {
                    fail(line, "indented, but not below a matrix");
                }
                Matrix& m = matrices[current];
                if (key == "rows" and not m.rows)
                {
                    m.rows = integer(value, line);
                }
                else if (key == "cols" and not m.cols)
                {
                    m.cols = integer(value, line);
                }
                else if (key == "data" and not m.data)
                {
                    if (value.empty() or value.front() != '[')
                    {
                        fail(line, "data: [ ... ] expected");
                    }
                    m.data.emplace();
                    pending_data = &*m.data;
                    pending = value.substr(1);
                    pending_line = line;
                    if (pending.find(']') == std::string::npos)
                    {
                        continue;
                    }
                }
                else
                {
                    fail(line, current + ": '" + key + "' unknown or twice");
                }
            }
            // a complete list: numbers up to ']', nothing after it
            const auto close = pending.find(']');
            if (not trim(pending.substr(close + 1)).empty())
            {
                fail(line, "text after ']'");
            }
            std::istringstream items(pending.substr(0, close));
            std::string item;
            while (std::getline(items, item, ','))
            {
                if (trim(item).empty() and items.eof() and pending_data->empty())
                {
                    break;  // []
                }
                pending_data->push_back(number(item, pending_line));
            }
            pending_data = nullptr;
            pending.clear();
        }
        if (pending_data)
        {
            fail(pending_line, "list without ']'");
        }
        for (const char* key : {"image_width", "image_height", "distortion_model"})
        {
            if (not scalars.count(key))
            {
                fail(line, std::string("'") + key + "' missing");
            }
        }
        for (const char* key :
             {"camera_matrix", "distortion_coefficients", "rectification_matrix", "projection_matrix"})
        {
            if (not matrices.count(key))
            {
                fail(line, std::string("'") + key + "' missing");
            }
        }
        check("camera_matrix", matrices["camera_matrix"], 3, 3, line);
        check("distortion_coefficients", matrices["distortion_coefficients"], 1, -1, line);
        check("rectification_matrix", matrices["rectification_matrix"], 3, 3, line);
        check("projection_matrix", matrices["projection_matrix"], 3, 4, line);
        CameraInfo info;
        info.width = static_cast<std::uint32_t>(integer(scalars["image_width"], line));
        info.height = static_cast<std::uint32_t>(integer(scalars["image_height"], line));
        info.distortion_model = scalars["distortion_model"];
        info.d = *matrices["distortion_coefficients"].data;
        std::copy_n(matrices["camera_matrix"].data->begin(), 9, info.k.begin());
        std::copy_n(matrices["rectification_matrix"].data->begin(), 9, info.r.begin());
        std::copy_n(matrices["projection_matrix"].data->begin(), 12, info.p.begin());
        return info;
    }

    CameraInfo CameraInfo::read_file(const std::string& path)
    {
        std::ifstream file(path);
        if (not file)
        {
            throw std::invalid_argument("calibration file " + path + ": cannot be read");
        }
        std::ostringstream text;
        text << file.rdbuf();
        try
        {
            return from_yaml(text.str());
        }
        catch (const std::invalid_argument& e)
        {
            throw std::invalid_argument(path + ": " + e.what());
        }
    }

    std::string CameraInfo::to_yaml(const std::string& camera_name) const
    {
        std::ostringstream out;
        out << "image_width: " << width << "\nimage_height: " << height << "\ncamera_name: " << camera_name << "\n";
        list(out, "camera_matrix", 3, 3, std::vector<double>(k.begin(), k.end()));
        out << "distortion_model: " << distortion_model << "\n";
        list(out, "distortion_coefficients", 1, static_cast<long>(d.size()), d);
        list(out, "rectification_matrix", 3, 3, std::vector<double>(r.begin(), r.end()));
        list(out, "projection_matrix", 3, 4, std::vector<double>(p.begin(), p.end()));
        return out.str();
    }

    void CameraInfo::write_file(const std::string& path, const std::string& camera_name) const
    {
        std::ofstream file(path);
        file << to_yaml(camera_name);
        if (not file)
        {
            throw std::runtime_error("calibration file " + path + ": cannot be written");
        }
    }
}  // namespace xblick
