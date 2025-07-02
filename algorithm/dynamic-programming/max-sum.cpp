/**
 * # problem
 * n個の整数a[0], a[1],..., a[n-1]が与えられる。これらの整数から何個かの整数を選んで総和をとったときの、総和の最大値を求めよ。また、何も選ばない場合の総和は0である。
 * Given n integers a[0], a[1],.., a[n-1], find the maximum possible sum that can be obtained by selecting some of these integers. If no integers are selected, the sum is 0.
 * 
 * # constraints
 * 1 <= n <= 100
 * -1000 <= a[i] <= 1000
 *
 * # examples
 * 1)
 * n = 3
 * a = (7, -6, 9)
 * answer = 16
 * 
 * 2)
 * n = 2
 * a = (-9, -16)
 * answer = 0
 */

#include <algorithm>
#include <iostream>

int main() {
    int n; // The number of inputs
    int a[10010]; // Input container

    int dp[10010]; // DP table

    std::cin >> n;
    for (int i = 0; i < n; ++i) {
        std::cin >> a[i];
    }

    dp[0] = 0;
    for (int i = 0; i < n; ++i) {
        dp[i + 1] = std::max(dp[i], dp[i] + a[i]);
    }

    std::cout << "Answer: " << dp[n] << std::endl;
}
