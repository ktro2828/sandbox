#include <deque>
#include <iostream>

template <typename T>
class queue {
 public:
  using size_type = typename std::deque<T>::size_type;
  using iterator = typename std::deque<T>::iterator;
  using const_iterator = typename std::deque<T>::const_iterator;

  explicit queue(size_t size) : data_(size, 0) {}
  queue(size_t size, T value) : data_(size, value) {}

  size_type size() const noexcept { return data_.size(); }

  void push_back(const T&& v) {
    data_.pop_front();
    data_.push_back(v);
  }

  void push_back(const T& v) {
    data_.pop_front();
    data_.push_back(v);
  }

  iterator begin() { return data_.begin(); }
  const_iterator begin() const { return data_.cbegin(); }
  const_iterator cbegin() const noexcept { return data_.cbegin(); }

  iterator end() {return data_.end();}
  const_iterator end() const {return data_.end();}
  const_iterator cend() const noexcept { return data_.cend(); }

 private:
  std::deque<T> data_;
};


template <typename T>
std::ostream & operator<<(std::ostream & os, const queue<T> & q) {
  os << "(";
  size_t i = 0;
  for (const auto & v : q) {
    os << v;
    if ( i != q.size() - 1) {
      os << ", ";
    }
    ++i;
  }
  os << ")";

  return os;
}

int main() {
  queue<int> q(5);

  std::cout << q << std::endl;
  for (int i = 1; i < 10; ++i) {
    q.push_back(i);
    std::cout << q << std::endl;
  }
}
