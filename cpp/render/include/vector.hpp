#ifndef VECTOR_H_
#define VECTOR_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cassert>
#include <iostream>


template <int N>
  class VectorNd
{
private:
  std::array<double, N> data_;

public:
  // Constrctor
  VectorNd(std::array<double, N> v) : data_(v) {};
  VectorNd(std::initializer_list<double> v) {
    size_t dims  = static_cast<size_t>(N);
    size_t n = std::min(dims, v.size());
    std::copy_n(v.begin(), n, data_.begin());
  }

  double& operator[](const int i) {
    assert(i>=0 && i < N);
    return data_[i];
  }

  double norm() const {return std::sqrt(*this * *this);}
  VectorNd<N> getNormalized() const {return *this / norm();}
  VectorNd<N>& normalize() {*this = *this / norm(); return *this;}

  // double dot(const VectorNd<N>& v) const;
  // double cross(const VectorNd<N>& v) const;

  // double getAngle(const VectorNd<N>& v) const;
  // double getCos(const VectorNd<N>& v) const;

  // VectorNd<N> getTransposed(int[] order) const;
  // VectorNd<N>& transpose(int[] order);

}; // class VectorNd

// template <int N>
// VectorNd<N> operator*(const VectorNd<N>& lhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator+(const VectorNd<N>& lhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator-(const VectorNd<N>& lhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator/(const VectorNd<N>& lhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator*(const double& rhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator*(const VectorNd<N>& lhs, const double& rhs);

// template <int N>
// VectorNd<N> operator+(const double& rhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator+(const VectorNd<N>& lhs, const double& rhs);

// template <int N>
// VectorNd<N> operator-(const double& rhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator-(const VectorNd<N>& lhs, const double& rhs);

// template <int N>
// VectorNd<N> operator/(const double& rhs, const VectorNd<N>& rhs);

// template <int N>
// VectorNd<N> operator/(const VectorNd<N>& lhs, const double& rhs);

// template <int N>
// VectorNd<N>& operator*=(const double& d);

// template <int N>
// VectorNd<N>& operator*=(const VectorNd<N>& v);

// template <int N>
// VectorNd<N>& operator+=(const double& d);

// template <int N>
// VectorNd<N>& operator+=(const VectorNd<N>& v);

// template <int N>
// VectorNd<N>& operator-=(const double& d);

// template <int N>
// VectorNd<N>& operator-=(const VectorNd<N>& v);

// template <int N>
// VectorNd<N>& operator/=(const double& d);

// template <int N>
// VectorNd<N>& operator/=(const VectorNd<N>& v);

// template <int N>
// std::ostream& operator<<(std::ostream& out, const VectorNd<T, N>& v);

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

typedef VectorNd<1> Vector1d;
typedef VectorNd<2> Vector2d;
typedef VectorNd<3> Vector3d;

#endif // VECTOR_H_
