#ifndef MYLIB_ITERATOR_HPP_
#define MYLIB_ITERATOR_HPP_

#include <iterator>

namespace mylib
{

template<typename T>
class iterator : public std::iterator<std::input_iterator_tag, T>
{
private:
    int counter_;
    T value_;

public:
    iterator(const int counter = 0) : counter_(counter), value_(0) {}

    iterator& operator++()
    {
        ++counter_;
        return *this;
    }

    T operator*() const { return value_; }

    bool operator!=(const iterator & x) const { return counter_ != x.counter_; }
}; // class iterator
} // namespace mylib

#endif // MYLIB_ITERATOR_HPP_