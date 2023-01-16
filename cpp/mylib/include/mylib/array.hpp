#pragma once

namespace mylib
{
    template <typename Array>
    class array_iterator
    {
    private:
        Array &a_;
        std::size_t i_;

    public:
        array_iterator(Array &a, std::size_t i) : a_(a), i_(i) {}

        array_iterator &operator+=(std::size_t n)
        {
            i_ += n;
            return *this;
        }
    }; // struct array_iterator_begin

    template <typename T, std::size_t N>
    class array
    {
    }; // class array
}
