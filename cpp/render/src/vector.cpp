#include <iostream>
#include <stdexcept>
#include "vector.hpp"

// operator overload
template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator*=(const double &d)
{
  for (int i = 0; i < N; ++i)
    data_[i] *= d;
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator*=(const VectorNd<T, N> &v)
{
  for (int i = 0; i < N; ++i)
    data_[i] *= v[i];
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator+=(const double &d)
{
  for (int i = 0; i < N; ++i)
    data_[i] += d;
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator+=(const VectorNd<T, N> &v)
{
  for (int i = 0; i < N; ++i)
    data_[i] += v[i];
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator-=(const double &d)
{
  for (int i = 0; i < N; ++i)
    data_[i] -= d;
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator-=(const VectorNd<T, N> &v)
{
  for (int i = 0; i < N; ++i)
    data_[i] -= v[i];
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator/=(const double &d)
{
  if (d == 0)
  {
    throw std::invalid_argument("zero division is invalid");
  }
  for (int i = 0; i < N; ++i)
    data_[i] /= d;
  return *this;
}

template <class T, int N>
VectorNd<T, N> &VectorNd<T, N>::operator/=(const VectorNd<T, N> &v)
{
  for (int i = 0; i < N; ++i)
  {
    if (v[i] == 0)
    {
      throw std::invalid_argument("zero division is invalid");
    }
    data_[i] /= v[i];
  }
  return *this;
}

// member functions
template <class T, int N>
double VectorNd<T, N>::dot(const VectorNd<T, N> &v) const
{
  double ret = 0;
  for (int i = 0; i < N; ++i)
  {
    ret += data_[i] * v[i];
  }
  return ret;
}

template <class T, int N>
VectorNd<T, N> VectorNd<T, N>::cross(const VectorNd<T, N> &v) const
{
  if (N != 3)
  {
    throw std::invalid_argument("cross must be computed for 3-dim vector");
  }

  VectorNd<T, 3> ret = {
      data_[1] * v[2] - data_[2] * v[1],
      data_[2] * v[0] - data_[0] * v[2],
      data_[0] * v[1] - data_[1] * v[0]};

  return ret;
}
