#include "vector.hpp"

template <int N>
VectorNd<N> operator*(const VectorNd<N>& lhs, const VectorNd<N>& rhs) {
  VectorNd<N> ret = lhs;
  for (int i=0; i<N; ++i) {
    ret[i] *= rhs;
  }
  return ret;
}

template <int N>
VectorNd<N> operator+(const VectorNd<N>& lhs, const VectorNd<N>& rhs) {
  VectorNd<N> ret = lhs;
  for (int i=0; i<N; ++i) {
    ret[i] += rhs;
  }
  return ret;
}

template <int N>
VectorNd<N> operator-(const VectorNd<N>& lhs, const VectorNd<N>& rhs) {
  VectorNd<N> ret = lhs;
  for (int i=0; i<N; ++i) {
    ret[i] -= rhs;
  }
  return ret;
}

template <int N>
VectorNd<N> operator/(const VectorNd<N>& lhs, const VectorNd<N>& rhs) {
  VectorNd<N> ret = lhs;
  for (int i=0; i<N; ++i) {
    ret[i] /= rhs;
  }
  return ret;
}
