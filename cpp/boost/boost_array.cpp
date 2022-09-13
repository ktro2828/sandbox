#include <iostream>
#include <boost/array.hpp>
#include <algorithm>


void disp(int x) { std::cout << x << ' '; }

int main()
{
  boost::array<int, 3> ar = {3, 1, 4};

  // 配列の要素数
  const std::size_t size = ar.size();
  std::cout << size << std::endl;

  // 添字による要素アクセス
  std::cout << ar[0] << std::endl;

  // イテレータ
  std::for_each(ar.begin(), ar.end(), disp);  

  boost::array<int, 3>::iterator it = std::find(ar.begin(), ar.end(), 1);
  if (it != ar.end()) {
    std::cout << *it << std::endl;
  }
}
