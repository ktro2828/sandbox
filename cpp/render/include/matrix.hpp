#ifndef MATRIX_H_
#define MATRIX_H_

#include <algorithm>
#include <array>
#include <cassert>
#include <stdexcept>

template <int N, int M>
class MatrixNd
{
private:
    std::array<std::array<double, M>, N> data_;

public:
    // Constructor
    MatrixNd(std::array<std::array<double, M>, N> data) : data_(data){};
    MatrixNd(std::initializer_list<std::initializer_list<double>> init)
    {
        size_t n = static_cast<size_t>(N);
        size_t m = static_cast<size_t>(M);

        size_t nrows = std::min(n, init.size());
        int i = 0;
        for (auto &it : init)
        {
            size_t ncols = it.size(); // initの先頭要素のサイズ = 列数
            std::copy_n(it.begin(), ncols, data_[i].begin());
            ++i;
        }
    }

    std::array<double, M> &operator[](const int i)
    {
        if (i < 0 || i > N)
        {
            throw std::invalid_argument("index out of range");
        }
        return data_[i];
    }
    const std::array<double, M> &operator[](const int i) const
    {
        if (i < 0 || i > N)
        {
            throw std::invalid_argument("index out of range");
        }
        return data_[i];
    }

    template <int L>
    MatrixNd<N, L> dot(const MatrixNd<M, L> &mat)
    {
        // TODO
        for (int i = 0; i < N; ++i)
        {
        }
        return *this;
    }

    // overload
    MatrixNd<N, M> &operator*=(const double &d);
    MatrixNd<N, M> &operator*=(const MatrixNd<N, M> &mat);
    MatrixNd<N, M> &operator+=(const double &d);
    MatrixNd<N, M> &operator+=(const MatrixNd<N, M> &mat);
    MatrixNd<N, M> &operator-=(const double &d);
    MatrixNd<N, M> &operator-=(const MatrixNd<N, M> &mat);
    MatrixNd<N, M> &operator/=(const double &d);
    MatrixNd<N, M> &operator/=(const MatrixNd<N, M> &mat);

}; // class MatrixNd

template <int N, int M>
std::ostream &operator<<(std::ostream &os, const MatrixNd<N, M> &mat)
{
    os << "(";
    for (int i = 0; i < N; ++i)
    {
        os << "(";
        for (int j = 0; j < M; ++j)
        {
            if (j != M - 1)
            {
                os << mat[i][j] << ", ";
            }
            else
            {
                os << mat[i][j] << ")";
            }
        }
        if (i != N - 1)
        {
            os << ",\n ";
        }
        else
        {
            os << ")";
        }
    }
    return os;
}
#endif // MATRIX_H_