#include <iostream>
#include <vector>
#include <algorithm>

void print(std::vector<int> &v) {
  std::cout << "[ ";
  std::for_each(v.begin(), v.end(), [](const int &n) {std::cout << n << " ";});
  std::cout << "]" << std::endl;
}

int main() {

  std::vector<int> x{1, 2, 3};

  print(x);
}
