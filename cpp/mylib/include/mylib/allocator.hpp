#ifndef MYLIB_ALLOCATOR_HPP_
#define MYLIB_ALLOCATOR_HPP_

#include <cstddef>
#include <cstdlib>

namespace mylib
{
template <class T>
class allocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;

public:
  allocator() noexcept {}
  allocator(const allocator &) noexcept {}

  template <class U>
  allocator(const allocator<U> &) noexcept
  {
  }

  ~allocator() {}

  allocator & operator=(const allocator &) = default;

  [[nodiscard]] pointer allocate(size_type n)
  {
    return reinterpret_cast<T *>(std::malloc(sizeof(T) * n));
  }

  void deallocate(pointer p, size_type n)
  {
    static_cast<void>(n);
    std::free(p);
  }

};  // class  allocator
}  // namespace mylib
#endif  // MYLIB_ALLOCATOR_HPP_
