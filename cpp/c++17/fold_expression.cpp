#include <iostream>
#include <string>

/*
Reference: 
https://cpprefjp.github.io/lang/cpp17/folding_expressions.html
 */

template<typename... Args>
auto sum(Args... args)
{
  return (args + ...);
}

template<typename... Args>
bool all(Args... args)
{
  return (... && args);
}

template<typename... Args>
void print_all(std::ostream& os, Args... args)
{
  (os << ... << args);
}


int main()
{
  std::cout << std::boolalpha;
  std::cout << sum(1, 2, 3, 4, 5) << "\n";
  std::cout << all(true, false, true) << "\n";
  print_all(std::cout, 1, 2, 3, "\n");
}
