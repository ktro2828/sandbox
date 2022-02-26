#include <iostream>
#include <stdexcept>
#include "vector.hpp"

// operator overload
template <int N>
VectorNd<N> &VectorNd<N>::operator*=(const double &d)
{
  for (int i = 0; i < N; ++i)
    data_[i] *= d;
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator*=(const VectorNd<N> &v)
{
  for (int i = 0; i < N; ++i)
    data_[i] *= v[i];
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator+=(const double &d)
{
  for (int i = 0; i < N; ++i)
    data_[i] += d;
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator+=(const VectorNd<N> &v)
{
  for (int i = 0; i < N; ++i)
    data_[i] += v[i];
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator-=(const double &d)
{
  for (int i = 0; i < N; ++i)
    data_[i] -= d;
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator-=(const VectorNd<N> &v)
{
  for (int i = 0; i < N; ++i)
    data_[i] -= v[i];
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator/=(const double &d)
{
  if (d == 0)
  {
    throw std::invalid_argument("zero division is invalid");
  }
  for (int i = 0; i < N; ++i)
    data_[i] /= d;
  return *this;
}

template <int N>
VectorNd<N> &VectorNd<N>::operator/=(const VectorNd<N> &v)
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
template <int N>
double VectorNd<N>::dot(const VectorNd<N> &v) const
{
  double ret = 0;
  for (int i = 0; i < N; ++i)
  {
    ret += data_[i] * v[i];
  }
  return ret;
}

template <int N>
VectorNd<N> VectorNd<N>::cross(const VectorNd<N>& v) const {
  if (N != 3) {
    throw std::invalid_argument("cross must be computed for 3-dim vector");
  }

  VectorNd<3> ret = {
    data_[1] * v[2] - data_[2] * v[1],
    data_[2] * v[0] - data_[0] * v[2],
    data_[0] * v[1] - data_[1] * v[0]
  };

  return ret;
}
