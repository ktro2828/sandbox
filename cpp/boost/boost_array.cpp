#include <iostream>
#include <boost/array.hpp>
#include <algorithm>


void disp(int x) { std::cout << x << ' '; }

int main()
{
  boost::array<int, 3> ar = {3, 1, 4};

  std::for_each(ar.begin(), ar.end(), disp);
}
