#ifndef VECTOR_H_
#define VECTOR_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cassert>
#include <iostream>
#include <stdexcept>

namespace geometry
{
  template <class T, int N>
  class VectorNd
  {
  private:
    std::array<T, N> data_;

  public:
    // Constrctor
    VectorNd() = default;
    VectorNd(std::array<T, N> v) : data_(v) {}
    VectorNd(std::initializer_list<T> v)
    {
      size_t dims = static_cast<size_t>(N);
      size_t n = std::min(dims, v.size());
      std::copy_n(v.begin(), n, data_.begin());
    }

    T &operator[](const int i)
    {
      assert(i >= 0 && i < N);
      return data_[i];
    }
    T operator[](const int i) const
    {
      assert(i >= 0 && i < N);
      return data_[i];
    }

    double norm() const { return std::sqrt(this->dot(*this)); }

    VectorNd<T, N> getNormalized() const { return *this / this->norm(); }
    void normalize() { *this /= this->norm(); }

    double getCos(const VectorNd<T, N> &v) const { return this->dot(v) / (this->norm() * v.norm()); }
    double getAngle(const VectorNd<T, N> &v) const { return std::acos(this->getCos(v)); }

    // VectorNd<T, N> getTransposed(const int[] &shape) const
    // {
    //   VectorNd<T, N> ret = {};
    //   for (int i = 0; i < N; ++i)
    //   {
    //     ret[i] = *this[shape[i]];
    //   }
    //   return ret;
    // }
    // void transpose(int[] order);

    bool all() const
    {
      for (int i = 0; i < N; ++i)
      {
        if (data_[i] == 0)
        {
          return false;
        }
      }
      return true;
    }

    bool any() const
    {
      for (int i = 0; i < N; ++i)
      {
        if (data_[i] != 0)
        {
          return true;
        }
      }
      return false;
    }

    int ndim() const { return N; }

    double dot(const VectorNd<T, N> &v) const;
    VectorNd<T, N> cross(const VectorNd<T, N> &v) const;

    // overload
    VectorNd<T, N> &operator*=(const double &d);
    VectorNd<T, N> &operator*=(const VectorNd<T, N> &v);
    VectorNd<T, N> &operator+=(const double &d);
    VectorNd<T, N> &operator+=(const VectorNd<T, N> &v);
    VectorNd<T, N> &operator-=(const double &d);
    VectorNd<T, N> &operator-=(const VectorNd<T, N> &v);
    VectorNd<T, N> &operator/=(const double &d);
    VectorNd<T, N> &operator/=(const VectorNd<T, N> &v);

  }; // class VectorNd

  typedef VectorNd<double, 1> Vector1d;
  typedef VectorNd<double, 2> Vector2d;
  typedef VectorNd<double, 3> Vector3d;

  template <class T, int N>
  VectorNd<T, N> operator*(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] *= rhs[i];
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator+(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] += rhs[i];
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator-(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] -= rhs[i];
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator/(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      if (rhs[i] == 0)
      {
        throw std::invalid_argument("zero division is invalid");
      }
      ret[i] /= rhs[i];
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator*(const double &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = rhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] *= lhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator*(const VectorNd<T, N> &lhs, const double &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] *= rhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator+(const double &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = rhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] += lhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator+(const VectorNd<T, N> &lhs, const double &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] += rhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator-(const double &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<T, N> ret = rhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] -= lhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator-(const VectorNd<T, N> &lhs, const double &rhs)
  {
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] -= rhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator/(const double &lhs, const VectorNd<T, N> &rhs)
  {
    if (lhs == 0)
    {
      throw std::invalid_argument("zero division is invalid");
    }
    VectorNd<T, N> ret = rhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] /= lhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<T, N> operator/(const VectorNd<T, N> &lhs, const double &rhs)
  {
    if (rhs == 0)
    {
      throw std::invalid_argument("zero division is invalid");
    }
    VectorNd<T, N> ret = lhs;
    for (int i = 0; i < N; ++i)
    {
      ret[i] /= rhs;
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<bool, N> operator==(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<bool, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = (lhs[i] == rhs[i]);
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<bool, N> operator<(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<bool, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = (lhs[i] < rhs[i]);
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<bool, N> operator<=(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<bool, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = (lhs[i] <= rhs[i]);
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<bool, N> operator>(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<bool, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = (lhs[i] > rhs[i]);
    }
    return ret;
  }

  template <class T, int N>
  VectorNd<bool, N> operator>=(const VectorNd<T, N> &lhs, const VectorNd<T, N> &rhs)
  {
    VectorNd<bool, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = (lhs[i] >= rhs[i]);
    }
    return ret;
  }

  template <class T, int N>
  std::ostream &operator<<(std::ostream &os, const VectorNd<T, N> &v)
  {
    os << "(";
    for (int i = 0; i < N; ++i)
    {
      if (i != N - 1)
      {
        os << v[i] << ", ";
      }
      else
      {
        os << v[i] << ")";
      }
    }
    return os;
  }

  template <class T, int N, int M>
  VectorNd<T, N> embed(const VectorNd<T, M> &v, T fill)
  {
    VectorNd<T, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = (i < M ? v[i] : fill);
    }
    return ret;
  }

  template <class T, int N, int M>
  VectorNd<T, N> proj(const VectorNd<T, M> &v)
  {
    VectorNd<T, N> ret;
    for (int i = 0; i < N; ++i)
    {
      ret[i] = v[i];
    }
    return ret;
  }
} // namespace geoemtry
#endif // VECTOR_H_
