#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

class Foo
{
  int sum_;
  std::mutex mtx_;

public:
  Foo() : sum_(0) {}
  static int calc(int x) { return 2 * x; }
  void parallel()
  {
    std::vector<std::thread> threads;
    for (int i = 0; i < 10000; ++i) {
      threads.emplace_back([i, this]() {
        mtx_.lock();
        sum_ += calc(i);
        mtx_.unlock();
      });
    }
    for (auto & t : threads) {
      t.join();
    }
  }
  int get_sum() const { return sum_; }
};

int main()
{
  std::vector<std::thread> threads;
  std::cout << "Max threads num: " << std::thread::hardware_concurrency() << std::endl;

  Foo foo;
  foo.parallel();
  std::cout << foo.get_sum() << std::endl;
}
