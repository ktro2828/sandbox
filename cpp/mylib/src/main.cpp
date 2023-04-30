#include "vector.hpp"

#include <iostream>

int main()
{
  mylib::vector<int> v(10, 1);
  std::cout << "Capacity: " << v.capacity() << std::endl;
  std::cout << "Size: " << v.size() << std::endl;
}
