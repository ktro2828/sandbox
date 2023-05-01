#include "vector.hpp"

#include <iostream>

int main()
{
  mylib::vector<int> v;
  std::cout << "Capacity: " << v.capacity() << std::endl;
  std::cout << "Size: " << v.size() << std::endl;
  std::cout << "begin: " << v.begin() << std::endl;
}
