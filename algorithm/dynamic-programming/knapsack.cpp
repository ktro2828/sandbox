/**
 * # problem
 * n個の品物があり、i番目の品物のそれぞれの重さと価値がweight[i],value[i]となっている(i=0,1,...,n-1)。
 * これらの品物から重さの総和がWを超えないように選んだときの、価値の総和の最大値を求めよ。
 * There are n items, where the weight and value of the i-th item are weight[i] and value[i], respectively (for i=0,1,...,n-1). 
 * Find the maximum possible total value that can be obtained by selecting items such that the total weight does not exceed W.
 * 
 * # constraints
 * 1 <= n <= 100
 * weight[i], value[i] are integer
 * 1 <= weight[i], value[i] <= 1000
 * 1 <= W <= 10000
 *
 * # examples
 * 1)
 * n = 6
 * (w, v) = (2, 3), (1, 2), (3, 6), (2, 1), (1, 3), (5, 85)
 * answer = 94
 */

#include <algorithm>
#include <iostream>


int main() {
    int n, W;
    int weight[110], value[110]; // input containers

    int dp[110][10010]; // DP table

    std::cin >> n >> W;
    for (int i = 0; i < n; ++i) {
        std::cin >> weight[i] >> value[i];
    }

    // init DP table
    for (int w = 0; w <= W; ++w) {
        dp[0][w] = 0;
    }

    for (int i = 0; i < n; ++i) {
        for (int w = 0; w <= W; ++w) {
            if (w >= weight[i]) {
                dp[i+1][w] = std::max(dp[i][w - weight[i]] + value[i], dp[i][w]);
            } else {
                dp[i+1][w] = dp[i][w];
            }
        }
    }

    std::cout << "answer: " << dp[n][W] << std::endl;

}
