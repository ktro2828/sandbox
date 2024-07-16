#include <iostream>
#include <sstream>
#include <string>
#include <vector>

template <typename T>
T uuid(const std::vector<uint8_t> & input);

template <>
uint16_t uuid<uint16_t>(const std::vector<uint8_t> & input) {
  uint16_t output = 0;
  for (size_t i = 0; i < input.size(); ++i) {
    output |= static_cast<uint16_t>(input.at(i)) << (i * 8);
  }
  return output;
}

template <>
std::string uuid<std::string>(const std::vector<uint8_t> & input) {
  std::ostringstream convert;
  for (const auto & v : input) {
    convert << static_cast<int>(v);
  }

  return convert.str();
}

int main() {
  std::vector<uint8_t> input = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

  auto out1 = uuid<uint16_t>(input);
  auto out2 = uuid<std::string>(input);

  std::cout << out1 << std::endl;
  std::cout << out2 << std::endl;
}
