# 構造化束縛

構造化束縛は，組やタプル，配列や構造体を分解して各要素を取り出す機能(>C++17)
組やタプル，配列に対しては，全要素分の展開先の変数が必要．

```cpp
#include <array>
#include <iostream>
#include <map>
#include <string>
#include <utility>


int main() {
	// 組
	std::pair<int, std::string> p{10, "Hello"};
	const auto &[a, b] = p;
	std::cout << a << ", " << b << std::endl;  // 10, Hello
	
	// map
	std::map<string, int> m{
		{"hoge", 1},
		{"fuga", 2},
		{"piyo", 3}
	};
	for (const auto &[key, value] : m) {
		std::cout << key << ": " << value << std::endl;
	}
	// hoge: 1
	// fuga: 2
	// piyo: 3

	// 配列
	std::array<int, 4> arr{1, 2, 3, 4};
	const auto &[x, y, z, w] = arr;
	// [ERROR!!] const auto &[x, y, z] = arr;
	
}
```

構造体に対して使う際は，構造体のメンバ変数分の展開先の変数が必要．
静的メンバ変数や定数，メンバ関数は無視される．

```cpp
struct Hoge {
	int age;
	std::string name;

	// staticは無視される
	static int static_foo;
	
	Hoge(int age_, std::string name_) : age(age_), name(name_) {
		static_foo = 123;
	}

	// メンバ関数は無視される
	void show_age() const {
		std::cout << age << std::endl;
	}

	void show_name() const {
		std::cout << name << std::endl;
	}
};


int main() {
	auto [age, name] = Hoge(10, "Bob");
}
```
