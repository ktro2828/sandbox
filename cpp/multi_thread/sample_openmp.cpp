#include <omp.h>

#include <iostream>

int main()
{
  const int threads_num = omp_get_max_threads();
  std::cout << "All Threads num: " << threads_num << std::endl;

#pragma omp parallel for
  for (int i = 0; i < 10; ++i) {
#pragma omp critical
    std::cout << "Thread No. " << omp_get_thread_num() << std::endl;
  }
  return 0;
}
