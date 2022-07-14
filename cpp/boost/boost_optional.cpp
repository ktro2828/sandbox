// Without boost::optional

// #include <iostream>
// #include <vector>
// #include <boost/assign/list_of.hpp>

// int* find_value(std::vector<int>& v, int value)
// {
//   for (std::size_t i = 0; i < v.size(); ++i) {
//     if (v[i] == value)
//       return &v[i];
//   }
//   return NULL;
// }

// int main()
// {
//   std::vector<int> v = boost::assign::list_of(1)(2)(3);

//   int* p = find_value(v, 3);
//   if (p) {
//     std::cout << *p << std::endl;
//   }
//   else {
//     std::cout << "該当なし" << std::endl;
//   }
// }

// With boost::optional
#include <iostream>
#include <vector>
#include <boost/assign/list_of.hpp>
#include <boost/optional.hpp>

boost::optional<int&> find_value(std::vector<int>& v, int value)
{
  for (std::size_t i = 0; i < v.size(); ++i)
    {
      if (v[i] == value)
	return v[i];
    }
  return boost::none;
}

int main()
{
  std::vector<int> v = boost::assign::list_of(1)(2)(3);
  boost::optional<int&> p = find_value(v, 4);
  if (p) {
    std::cout << p.get() << std::endl;
  }
  else {
    std::cout << "該当なし" << std::endl;
  }
}
