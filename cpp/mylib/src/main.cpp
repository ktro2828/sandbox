#include "vector.hpp"

#include <iostream>

template <typename T>
void debug(const mylib::vector<T> & v)
{
  std::cout << "(";
  for (size_t i = 0; i < v.size(); ++i) {
    std::cout << v.at(i);
    if (i != v.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << ")";
}

int main()
{
  mylib::vector<int> v(10);
  std::cout << "Capacity: " << v.capacity() << std::endl;
  std::cout << "Size: " << v.size() << std::endl;
  std::cout << "begin: " << v.begin() << std::endl;
  for (int i = 0; i < 10; ++i) {
    v.emplace_back(i);
  }

  debug<int>(v);
}
