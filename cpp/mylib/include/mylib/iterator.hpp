//   class iterator
//   {
//     using difference_type = int;
//     using value_type = vector::value_type;
//     using pointer = vector::pointer;
//     using reference = vector::reference;
//     using iterator_category = std::random_access_iterator_tag;

//   private:
//     value_type * ptr_;

//   public:
//     iterator() noexcept : ptr_(nullptr) {}
//     iterator(vector * base, difference_type index) noexcept : ptr_(base->first_ + index) {}
//     iterator(const iterator & i) : ptr_(i.ptr_) {}

//     iterator & operator++()
//     {
//       ptr_++;
//       return *this;
//     }

//     iterator operator++(int)
//     {
//       iterator ret = *this;
//       ptr_++;
//       return ret;
//     }

//     iterator operator+(const difference_type i) const { return iterator(*this) += i; }

//     iterator & operator+(const difference_type i)
//     {
//       ptr_ += i;
//       return *this;
//     }

//     iterator & operator--()
//     {
//       ptr_--;
//       return *this;
//     }

//     iterator operator--(int)
//     {
//       iterator ret = *this;
//       ptr_--;
//       return ret;
//     }

//     iterator operator-(const difference_type i) const { return iterator(*this) -= i; }

//     iterator & operator-(const difference_type i)
//     {
//       ptr_ -= i;
//       return *this;
//     }

//     difference_type operator-(const iterator & iter) const { return ptr_ - iter.ptr_; }

//     reference operator*() const { return *ptr_; }

//     reference operator[](const difference_type i) const { return *(*this + i); }

//     bool operator<(const iterator & iter) const { return ptr_ < iter.ptr_; }

//     bool operator<=(const iterator & iter) const { return ptr_ <= iter.ptr_; }

//     bool operator==(const iterator & iter) const { return ptr_ == iter.ptr_; }

//     bool operator>(const iterator & iter) const { return ptr_ > iter.ptr_; }

//     bool operator>=(const iterator & iter) const { return ptr_ >= iter.ptr_; }

//     bool operator!=(const iterator & iter) const { return ptr_ != iter.ptr_; }

//   };  // class iterator

//   class const_iterator
//   {
//     using difference_type = int;
//     using value_type = vector::value_type;
//     using pointer = const vector::pointer;
//     using reference = const vector::reference;
//     using iterator_category = std::random_access_iterator_tag;

//   private:
//     pointer ptr_;

//   public:
//     const_iterator() noexcept : ptr_(nullptr) {}
//     const_iterator(vector * base, difference_type index) noexcept : ptr_(base->first_ + index) {}
//     const_iterator(const iterator & iter) : ptr_(iter.ptr_) {}
//     const_iterator(const const_iterator & iter) : ptr_(iter.ptr_) {}

//     const_iterator & operator++()
//     {
//       ptr_++;
//       return *this;
//     }

//     const_iterator operator++(difference_type)
//     {
//       const_iterator ret = *this;
//       ptr_++;
//       return ret;
//     }

//     const_iterator operator+(const difference_type i) const { return const_iteator(*this) += i; }

//     const_iterator & operator+=(const difference_type i)
//     {
//       ptr_ += i;
//       return *this;
//     }

//     const_iterator & operator--()
//     {
//       ptr_--;
//       return *this;
//     }

//     const_iterator operator--(difference_type)
//     {
//       const_iterator ret = *this;
//       ptr_--;
//       return ret;
//     }

//     const_iterator operator-(const difference_type i) const { return const_iterator(*this) -= i; }

//     difference_type operator-(const const_iterator & iter) const { return ptr_ - iter.ptr_; }

//     const_iterator & operator-=(const difference_type i)
//     {
//       ptr_ -= i;
//       return *this;
//     }

//     reference operator*() const { return *ptr_; }

//     reference operator[](const difference_type i) const { return *(*this + i); }

//     bool operator<(const const_iterator & iter) const { return ptr_ < iter.ptr_; }

//     bool operator<=(const const_iterator & iter) const { return ptr_ <= iter.ptr_; }

//     bool operator==(const const_iterator & iter) const { return ptr_ == iter.ptr_; }

//     bool operator>(const const_iterator & iter) const { return ptr_ > iter.ptr_; }

//     bool operator>=(const const_iterator & iter) const { return ptr_ >= iter.ptr_; }

//     bool operator!=(const const_iterator & iter) const { return ptr_ != iter.ptr_; }
//   };  // class const_iterator

//   using reverse_iterator = std::reverse_iterator<iterator>;
//   using const_reverse_iterator = std::reverse_iterator<const_iterator>;