#include "matrix.hpp"

// operator oveload
template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator*=(const double &d)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] *= d;
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator*=(const MatrixNd<N, M> &mat)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] *= mat[i][j];
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator+=(const double &d)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] += d;
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator+=(const MatrixNd<N, M> &mat)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] += mat[i][j];
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator-=(const double &d)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] -= d;
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator-=(const MatrixNd<N, M> &mat)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] -= mat[i][j];
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator/=(const double &d)
{
    if (d == 0)
    {
        throw std::invalid_argument("zero division is invalid");
    }
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            data_[i][j] /= d;
        }
    }
    return *this;
}

template <int N, int M>
MatrixNd<N, M> &MatrixNd<N, M>::operator/=(const MatrixNd<N, M> &mat)
{
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            if (mat[i][j] == 0)
            {
                throw std::invalid_argument("zero division is invalid");
            }
            data_[i][j] /= mat[i][j];
        }
    }
    return *this;
}
