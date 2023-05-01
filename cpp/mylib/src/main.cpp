#include "allocator.hpp"
#include "vector.hpp"

#include <iostream>

template <typename T, class Alloc>
void debug(const mylib::vector<T, Alloc> & v)
{
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

  std::cout << "Capacity: " << v.capacity() << std::endl;
  std::cout << "Size: " << v.size() << std::endl;
  std::cout << "begin: " << v.begin() << std::endl;
  std::cout << "end: " << v.end() << std::endl;
  std::cout << v.end() - v.begin() << std::endl;
  for (int i = 0; i < 10; ++i) {
    v.emplace_back(i);
  }

  debug(v);
}
