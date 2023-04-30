#ifndef MYLIB_VECTOR_HPP_
#define MYLIB_VECTOR_HPP_

#include <memory>
namespace mylib
{
template <typename T, typename Alloc = std::allocator<T>>
class vector
{
private:
  static_assert(
    std::is_same<T, typename Alloc::value_type>::value,
    "The allocator value must be matched the vector value type.");
  static_assert(!std::is_const<T>::value, "Const elements are forbidden.");

  using traits = std::allocator_traits<Alloc>;
  using value_type = T;
  using allocator_type = Alloc;
  using size_type = unsigned int;
  using difference_type = int;
  using reference = T &;
  using const_reference = const T &;
  using pointer = typename traits::pointer;
  using const_pointer = typename traits::const_pointer;

  // Helper functions
  pointer allocate(size_type n) { return traits::allocate(alloc_, n); }
  void deallocate() { traits::deallocate(alloc_, first_, capacity_); }
  void construct(pointer ptr) { traits::construct(alloc_, ptr); }
  void construct(pointer ptr, const_reference value) { traits::construct(alloc_, ptr, value); }
  void construct(pointer ptr, value_type && value)
  {
    traits::construct(alloc_, ptr, std::move(value));
  }
  void destroy(pointer ptr) { traits::destroy(alloc_, ptr); }

private:
  pointer first_ = nullptr, last_ = nullptr;
  size_type size_ = 0, capacity_ = 1;
  Alloc alloc_;

public:
  vector() : vector(allocator_type()) {}
  explicit vector(const allocator_type & alloc) noexcept : alloc_(alloc) {}

  explicit vector(size_type n, const allocator_type & alloc = allocator_type()) : alloc_(alloc)
  {
    while (capacity_ < n) {
      capacity_ *= 2;
    }
    first_ = allocate(capacity_);
  }

  explicit vector(
    size_type n, const_reference value, const allocator_type & alloc = allocator_type())
  : alloc_(alloc)
  {
    while (capacity_ < n) {
      capacity_ *= 2;
    }

    for (size_type i = 0; i < n; ++i) {
      emplace_back(value);
    }
  }

  template <class... Args>
  void emplace_back(Args &&... args)
  {
    if (size_ == capacity_) {
    }
    construct(first_ + size_, std::forward<Args>(args)...);
    size_++;
  }

  /**
     @brief Deconstruct object
   */
  ~vector()
  {
    clear();
    deallocate();
  }

  /**
     @brief Copy constructor
   */
  vector & operator=(const vector & v)
  {
    clear();
    deallocate();
    size_ = v.size_;
    capacity_ = 1;
    while (capacity_ < size_) {
      capacity_ *= 2;
    }
    first_ = allocate(capacity_);
    for (size_type i = 0; i < size_; ++i) {
      construct(first_ + i, *(v.first_ + i));
    }
  }

  /**
     @brief Copy constructor by move
   */
  vector & operator=(vector && v)
  {
    clear();
    deallocate();
    *this = std::move(v);
    return *this;
  }

  void reserve(size_type s)
  {
    if (capacity_ < s) {
      pointer ptr = allocate(s);
      pointer old_first = first_;
      size_type old_capacity = capacity_;
      first_ = ptr;
      capacity_ = 1;
    }
  }

  /**
     @brief Resize vector
   */
  void resize(size_type s)
  {
    if (s < size_) {
      auto diff = size_ - s;
      for (size_type i = size_; i < size_; --i) {
        destroy(first_ + i);
      }
    }
  }

  /**
     @brief Clear data
   */
  void clear()
  {
    for (size_type i = 0; i < size_; ++i) {
      destroy(first_ + i);
    }
  }

  size_type capacity() const { return capacity_; }
  size_type size() const { return size_; }

public:
  class iterator
  {
    using difference_type = int;
    using value_type = vector::value_type;
    using pointer = vector::pointer;
    using reference = vector::reference;
    using iterator_category = std::random_access_iterator_tag;

  private:
    value_type * ptr_;

  public:
    iterator() noexcept : ptr_(nullptr) {}
    iterator(vector * base, difference_type index) noexcept : ptr_(base->first_ + index) {}
    iterator(const iterator & i) : ptr_(i.ptr_) {}

    iterator & operator++()
    {
      ptr_++;
      return *this;
    }

    iterator operator++(int)
    {
      iterator ret = *this;
      ptr_++;
      return ret;
    }

    iterator operator+(const difference_type i) const { return iterator(*this) += i; }

    iterator & operator+(const difference_type i)
    {
      ptr_ += i;
      return *this;
    }

    iterator & operator--()
    {
      ptr_--;
      return *this;
    }

    iterator operator--(int)
    {
      iterator ret = *this;
      ptr_--;
      return ret;
    }

    iterator operator-(const difference_type i) const { return iterator(*this) -= i; }

    iterator & operator-(const difference_type i)
    {
      ptr_ -= i;
      return *this;
    }

  };  // class iterator
};    // class vector
}  // namespace mylib

#endif  // MYLIB_VECTOR_HPP_
