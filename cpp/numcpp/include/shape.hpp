#pragma once

#include <iostream>
#include <string>

#include "types.hpp"

namespace numcpp
{
    class Shape
    {
    public:
        uint32 rows{0};
        uint32 cols{0};

        // Constructor
        Shape() = default;
        Shape(const uint32 dtype) : rows(dtype), cols(dtype) {}
        Shape(const uint32 rows_, const uint32 cols_) : rows(rows_), cols(cols_) {}

        bool operator==(const Shape &other) const
        {
            return rows == other.rows && cols == other.cols;
        }

        bool operator!=(const Shape &other) const
        {
            return !(*this == other);
        }

        uint32 size() const
        {
            return rows * cols;
        }

        std::string str() const
        {
            std::string out = "[" + std::to_string(rows) + ", " + std::to_string(cols) + "]\n";
            return out;
        }

        void print() const
        {
            std::cout << *this;
        }

        friend std::ostream &operator<<(std::ostream &os, const Shape &shape)
        {
            os << shape.str();
            return os;
        }
    }; // class Shape
} // namespace numcpp
