# `std::ostringstream` vs `std::stringstream`

## `std::ostringstream`

- 文字列の出力に特化いる、つまり書き込みのみ可能で、読み込みはできない。
- 主に文字列の組み立てやログの出力などで使用される。

```cpp
#include <iostream>
#include <sstream>

int main() {
    std::ostringstream oss;
    oss << "Hello, " << "World!";  // 書き込みのみ

    std::string result = oss.str();

    std::cout << "Result: " << result << std::endl;  // Result: Hello, World!
}
```

## `std::stringstream`

- `std::istream`および`std::ostream`両方の機能を提供する、つまり読み込むと書き込みの両方が可能。
- 例えば、`std::stringstream`を使用して文字列から数値を読み取り、また別の文字列に数値を書き込むことができる。

```cpp
#include <iostream>
#include <sstream>

int main() {
    std::stringstream ss;
    ss << "123"; // 書き込み

    int number;
    ss >> number; // 読み込み

    std::cout << "Read number: " << number << std::endl;  // Read number: 123    
}
```