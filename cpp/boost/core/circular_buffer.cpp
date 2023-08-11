// boost::circular_bufferは循環バッファのライブラリ
// FIFO(First In First Out)アルゴリズムが実現可能
// 実際にメモリ空間のコピーが発生しているのではなく，boost::circular_bufferが開始ポインタの位置を循環させている．

/*
メモリアドレスイメージ

 [0] [1] [2] 
 ___ ___ ___ 
|___|___|___|

push_front( 'a' )
 ___ ___ ___ 
|_a_|___|___|

push_front( 'b' )
 ___ ___ ___ 
|_b_|_a_|___|

push_front( 'c' )
 ___ ___ ___ 
|_c_|_b_|_a_|

push_front( 'd' )
 ___ ___ ___ 
|_d_|_c_|_b_|  <--- 'a'が消える
*/


#include <iostream>
#include <algorithm>
#include <boost/circular_buffer.hpp>

void disp(char x) { std::cout << x << ' '; }

int main()
{
  boost::circular_buffer<char> c_buf(3);

  c_buf.push_front('a');
  std::for_each(c_buf.begin(), c_buf.end(), disp);
  std::cout << std::endl;

  c_buf.push_front('b');
  std::for_each(c_buf.begin(), c_buf.end(), disp);
  std::cout << std::endl;

  c_buf.push_front('c');
  std::for_each(c_buf.begin(), c_buf.end(), disp);
  std::cout << std::endl;

  c_buf.push_front('d');
  std::for_each(c_buf.begin(), c_buf.end(), disp);
  std::cout << std::endl;
}
