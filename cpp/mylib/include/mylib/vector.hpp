#ifndef MYLIB_VECTOR_HPP_
#define MYLIB_VECTOR_HPP_

#include <iterator>
#include <memory>
#include <stdexcept>

namespace mylib
{
template <typename T, typename Alloc = std::allocator<T>>
class vector
{
public:
  using traits = std::allocator_traits<Alloc>;
  using value_type = T;
  using size_type = unsigned int;
  using pointer = typename traits::pointer;
  using const_pointer = typename traits::const_pointer;
  using reference = T &;
  using const_reference = const T &;
  // allocator
  using allocator_type = Alloc;
  // iterator
  using iterator = T*;
  using const_iterator = const T*;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

private:
  pointer first_;
  pointer last_;
  pointer reserved_last_;
  Alloc alloc_;

  // === Validation ===
  static_assert(
    std::is_same<T, typename Alloc::value_type>::value,
    "The allocator value must be matched the vector value type.");
  static_assert(!std::is_const<T>::value, "Const elements are forbidden.");

  // === Helper functions ===
  pointer allocate(size_type n) { return traits::allocate(alloc_, n); }
  void deallocate() { traits::deallocate(alloc_, first_, capacity()); }
  void construct(pointer ptr) { traits::construct(alloc_, ptr); }
  void construct(pointer ptr, const_reference value) { traits::construct(alloc_, ptr, value); }
  void construct(pointer ptr, value_type && value)
  {
    traits::construct(alloc_, ptr, std::move(value));
  }
  void destroy(pointer ptr) { traits::destroy(alloc_, ptr); }
  void destroy_until(reverse_iterator rend)
  {
    for (auto riter = rbegin(); riter != rend; ++riter, --last_) {
      destroy(&*riter);
    }
  }

public:
  // === Constructor ===
  vector() : vector(allocator_type()) {}

  explicit vector(const allocator_type & alloc) noexcept : alloc_(alloc) {}

  explicit vector(size_type n, const allocator_type & alloc = allocator_type()) : alloc_(alloc)
  {
    resize(n);
  }

  explicit vector(
    size_type n, const_reference value, const allocator_type & alloc = allocator_type())
  : alloc_(alloc)
  {
    resize(n, value);
  }

  // === Destructor ===
  ~vector()
  {
    clear();
    deallocate();
  }

  // === Copy-constructor ==
  vector & operator=(const vector & v)
  {
    clear();
    deallocate();
    resize(v.size());
    first_ = allocate(capacity());
    for (size_type i = 0; i < size(); ++i) {
      construct(first_ + i, *(v.first_ + i));
    }
  }

  vector & operator=(vector && v)
  {
    clear();
    deallocate();
    *this = std::move(v);
    return *this;
  }

  // === Iterator ===
  iterator begin() const noexcept { return first_; }
  const_iterator cbegin() const noexcept { return first_; }
  reverse_iterator rbegin() noexcept { return reverse_iterator(last_); }
  const_iterator crbegin() const noexcept { return const_reverse_iterator(last_); }
  iterator end() const noexcept { return last_; }
  const_iterator end() const noexcept { return last_; }
  const_iterator cend() const noexcept { return last_; }
  reverse_iterator rend() noexcept { return reverse_iterator(first_); }
  const_reverse_iterator rend() const noexcept { return const_reverse_iterator(first_); }
  const_reverse_iterator crend() const noexcept { return const_reverse_iterator(first_); }

  // === Size ===
  size_type capacity() const noexcept { return reserved_last_ - first_; }
  size_type size() const { return end() - begin();} 
  bool empty() const { return begin()== end(); }

  void reserve(size_type sz) {
    if (sz <= capacity()) {
      return;
    }
    auto ptr = allocate(sz);

    auto old_first = first_;
    auto old_last = last_;
    auto old_capacity = capacity();

    first_= ptr;
    last_ = first_;
    reserved_last_ = first_ + sz;
    
    for (auto old_iter = old_first; old_iter != old_last; ++old_iter, ++last_) {
      construct(last_, std::move(*old_iter));
    }

    for (auto riter = reverse_iterator(old_last), rend = reverse_iterator(old_first); riter != rend; ++riter)
    {
      destroy(&*riter);
    }

    traits::deallocate(alloc_, old_first, old_capacity);
  }

  void resize(size_type sz)
  {
    if (sz < size()) {
      auto diff = size() - sz;
      destroy_until(rbegin() + diff);
      last_ = first_ + sz;
    } else if (sz > size()){
      reserve(sz);
      for (; last_ != reserved_last_; ++last_) {
        construct(last_);
      }
    }
  }

  void resize(size_type sz, const_reference value) {
    if (sz < size()) {
      auto diff = size() - sz;
      destroy_until(rbegin() + diff);
      last_ = first_ + sz;
    } else if (sz > size()){
      reserve(sz);
      for (; last_ != reserved_last_; ++last_) {
        construct(last_, value);
      }
    }
  }

  // === Access to the elements ===
  reference operator[](const size_type i) { return first_[i]; }
  reference at(const size_type i)
  {
    if (i >= size()) {
      throw std::out_of_range("Index %d is out of range.", i);
    }
    return first_[i];
  }
  reference front() {
    return first_;
  }
  reference back() { return last_ - 1;}
  pointer data() { return first_; }

  // === Update container ===
  template <class... Args>
  void emplace_back(Args &&... args)
  {
    if (size() + 1 > capacity()) {
      size_type c = size();
      if (c == 0) {
        c = 1;
      } else {
        c *= 2;
      }
      reserve(c);
    }
    construct(last_, std::forward<Args>(args)...);
    ++last_;
  }

  void push_back(const value_type value) { emplace_back(value); }

  void clear() { destroy_until(rend()); }

};  // class vector
}  // namespace mylib

#endif  // MYLIB_VECTOR_HPP_
