#include MYLIB_VECTOR_HPP_
#define MYLIB_VECTOR_HPP_

#include "allocator.hpp"
#include "iterator.hpp"

namespace mylib
{
template<typename T, typename Allocator = allocator<T>>
class vector
{
private:
    T* first_;
    T* last_;

public:
    vector(std::size_t n = 0, Allocator a = Allocator()) {}
    ~vector();

    vector(const vector & v) {}
    vector &operator=(const vector &v) {}

    T &operator[](const std::size_t i) noexcept
    {
    }

    T &at(const std::size_t i) noexcept
    {
        
    }

    void push_back(const T &x) {}

    iterator<T> begin() noexcept {}
    iterator<T> end() noexcept {}
}; // class vector
} // namespace mylib

#endif // MYLIB_VECTOR_HPP_
