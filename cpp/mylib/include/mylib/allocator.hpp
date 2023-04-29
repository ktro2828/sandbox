#ifndef MYLIB_ALLOCATOR_HPP_
#define MYLIB_ALLOCATOR_HPP_

#include <cstddef>
#include <cstdlib>

namespace mylib {
template <class T>
class allocator
{
private:
public:
allocator() noexcept {}
allocator(const allocator &) noexcept {}

template <class U>
allocator(const allocator<U> &) noexcept {}

~allocator() {}

allocator& operator=(const allocator&) = default;

[[nodiscard]] T* allocate(std::size_t n)
{
    return reinterpret_cast<T*>(std::malloc(sizeof(T) * n));
}

void deallocate(T *p, size_t n)
{
    static_cast<void>(n);
    std::free(p);
}

}; // class  allocator
} // namespace mylib
#endif // MYLIB_ALLOCATOR_HPP_