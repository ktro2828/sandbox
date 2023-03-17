/*
Reference:
https://cpprefjp.github.io/lang/cpp17/structured_bindings.html
 */

#include <iostream>
#include <utility>
#include <tuple>
#include <string>

std::pair<int, std::string> f()
{
  return {3, "Hello"};
}

std::tuple<int, std::string, double> g()
{
  return {1, "World", 3.14};
}

int main()
{
  {
    const auto [id, message] = f();
    std::cout << id << std::endl;
    std::cout << message << std::endl;
  }

  {
    const auto [id, message, value] = g();
    std::cout << id << std::endl;
    std::cout << message << std::endl;
    std::cout << value << std::endl;
  }

  {
    auto t = g();
    auto &[id, message, value] = t;
    message = "My World";

    std::cout << id << std::endl;
    std::cout << message << std::endl;
    std::cout << value << std::endl;
  }
  
}
