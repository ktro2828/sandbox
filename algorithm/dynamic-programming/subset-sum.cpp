/**
 * # problem
 * n個の正の整数a[0], a[1],..., a[n-1]と正の整数Aが与えられる。
 * これらの整数から何個かの整数を選んで総和がAになるようにすることが可能か判定せよ。
 * 可能ならばYESと出力し、不可能ならばNOと出力せよ。
 * Given n positive integers a[0], a[1],..., a[n-1] and a positive integer A,
 * determine whether it is possible to select some of these integers such that their sum equals A.
 * If it is possible, output YES; otherwise, output NO.
 *
 * # constraints
 * 1 <= n <= 100
 * 1 <= a[i] <= 1000
 * 1 <= A <= 10000
 *
 * # examples
 * 1)
 * n = 3
 * a = (7, 5, 3)
 * A = 10
 * answer: YES
 * 
 * 2)
 * n = 2
 * a = (9, 7)
 * A = 6
 * answer: NO
 */

#include <cstring>

#include <iostream>
int main() {
    int n, A;
    int a[110]; // input container

    bool dp[110][10010]; // DP container

    std::cin >> n >> A;
    for (int i = 0; i < n; ++i) {
        std::cin >> a[i];
    }

    memset(dp, 0, sizeof(dp));
    dp[0][0] = true;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= A; ++j) {
            dp[i + j][j] |= dp[i][j];
            if (j >= a[i]) {
                dp[i + 1][j] |= dp[i][j - a[i]];
            }
        }
    }

    if (dp[n][A]) {
        std::cout << "YES" << std::endl;
    } else {
        std::cout << "NO" << std::endl;
    }
}
