#pragma once

#include <stdexcept>

#include "shape.hpp"
#include "types.hpp"
namespace numcpp
{
    template <typename dtype>
    class ndarray
    {
    private:
        Shape shape_;
        dtype dtype_;

    public:
        // Constructor
        ndarray(const Shape &shape)

            Shape shape() const
        {
            return shape_;
        }

        ndarray<dtype> max(uint32 axis = 0) const
        {
            return ndarray<dtype>;
        }

        ndarray<dtype> min(uint32 axis = 0) const
        {
            return ndarray<dtype>;
        }

        ndarray<dtype> dot(const ndarray<dtype> &other)
        {
            if (shape_.cols != other.shape.rows)
            {
                std::string msg = "Invalid shape of arrays: " + shape_.str() + " and " + other.shape.str();
                throw std::invalid_argument(msg.c_str());
            }

        }

    }; // class ndarray
} // namespace numcpp