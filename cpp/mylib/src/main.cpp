#include "allocator.hpp"
#include "vector.hpp"

#include <algorithm>
#include <iostream>

template <typename T, class Alloc>
void debug(const mylib::vector<T, Alloc> & v)
{
  std::cout << "Capacity: " << v.capacity() << std::endl;
  std::cout << "Size: " << v.size() << std::endl;
  std::cout << "(";
  for (size_t i = 0; i < v.size(); ++i) {
    std::cout << v.at(i);
    if (i != v.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << ")" << std::endl;
}

int main()
{
  mylib::vector<int, mylib::allocator<int>> v(10);
  debug(v);

  mylib::vector<int, mylib::allocator<int>> v2;
  v2 = v;

  debug(v2);

  std::cout << v.end() - v.begin() << std::endl;
  for (int i = 0; i < 10; ++i) {
    v.emplace_back(i);
  }

  mylib::vector<int>::iterator max_iter = std::max_element(v.begin(), v.end());
  int max_value = *max_iter;
  std::cout << "Max: " << max_value << std::endl;

  debug(v);

  mylib::vector<int> v3;
  debug(v3);
  v3.reserve(15);
  debug(v3);
  v3.resize(12);
  debug(v3);

  mylib::vector<int> v4 = {1, 2, 3, 4};
  debug(v4);
}
