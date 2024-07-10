#include <iostream>
#include <string>

namespace foo {

struct Foo {
  std::string name = "foo";
};

void check_foo(const foo::Foo &foo) { std::cout << foo.name << std::endl; }
// void check_bar(const foo::bar::Bar &bar) { std::cout << bar.name << std::endl; } // Compile ERROR!

namespace bar {
struct Bar {
  std::string name = "bar";
};

void check_bar(const Bar &bar) { std::cout << bar.name << std::endl; }
// void check_bar(const foo::bar::Bar &bar) { std::cout << bar.name << std::endl; } // OK!
// void check_bar(const bar::Bar &bar) { std::cout << bar.name << std::endl; } // OK!
void check_foo(const foo::Foo &foo) { std::cout << foo.name << std::endl; }
}
} // namespace foo

int main() {
  foo::Foo foo;
  foo::check_foo(foo);

  foo::bar::Bar bar;
  foo::bar::check_foo(foo);
  foo::bar::check_bar(bar);
}
