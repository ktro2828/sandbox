#include <iostream>

#include "vector.cpp"
#include "matrix.cpp"

using namespace std;

int main()
{
  VectorNd<double, 3> v1 = {1, 2, 3};
  VectorNd<double, 3> v2 = {2, 2, 2};

  Vector1d v1d = {1};
  Vector2d v2d = {2, 3};
  Vector3d v3d = {2, 3, 4};

  cout << "construct" << endl;
  v1 = v1 * v2;
  cout << "v1 * v2 = " << v1 << endl;
  v1 *= v2;

  v1 = v1 + v2;
  cout << "v1 + v2 = " << v1 << endl;
  v1 += v2;

  v1 = v1 - v2;
  cout << "v1 - v2 " << v1 << endl;
  v1 -= v2;

  v1 = v1 / v2;
  cout << "v1 / v2 = " << v1 << endl;
  v1 /= v2;

  double d = v1.dot(v2);
  cout << "v1 * v2 = " << d << endl;
  v1 = v1.cross(v2);
  cout << "v1 x v2 = " << v1 << endl;

  double norm = v1.norm();
  cout << "v1.norm() = " << norm << endl;

  VectorNd<double, 3> v3 = v1.getNormalized();
  cout << "v1.getNormalized() = " << v3 << endl;
  cout << "v1 = " << v1 << endl;
  v1.normalize();
  cout << "v1.normalize() = " << v1 << endl;

  int ndim = v1.ndim();
  cout << "v1.ndim() = " << ndim << endl;

  double cos = v1.getCos(v2);
  double theta = v1.getAngle(v2);
  cout << "v1.getCos(v2) = " << cos << endl;
  cout << "v1.getAngle(v2) = " << theta << endl;

  // int order[3] = {2, 1, 3};
  // VectorNd<double, 3> v4 = v1.getTransposed(order);
  // cout << "v1 = " << v1 << endl;
  // cout << "v1.getTransposed({2, 1, 3}) = " << v4 << endl;

  bool all = v1.all();
  cout << "v1.all() = " << all << endl;
  bool any = v1.any();
  cout << "v1.any() = " << any << endl;

  VectorNd<bool, 3> flagv = v1 == v2;
  cout << "v1 == v2 = " << flagv << endl;
  cout << "(v1 == v2).all() = " << flagv.all() << endl;
  cout << "(v1 == v2).any() = " << flagv.any() << endl;

  Vector3d v;
  cout << v << endl;

  MatrixNd<2, 2> m1 = {{2, 2}, {1, 1}};
  MatrixNd<2, 2> m2 = {{2, 2}, {1, 1}};

  m1 += m2;
  cout << "m 1+= m2 = \n"
       << m1 << endl;
  m1 -= m2;
  cout << "m1 -= m2 = \n"
       << m1 << endl;
  m1 *= m2;
  cout << "m1 *= m2 = \n"
       << m1 << endl;
  m1 /= m2;
  cout << "m1 /= m2 = \n"
       << m1 << endl;

  MatrixNd<2, 2> m3 = {};
  m3 = m1.dot(m2);
  cout << "m1.dot(m2) = " << m3 << endl;
  m3 = dot(m1, m2);
  cout << "dot(m1, m2) = " << m3 << endl;
}
