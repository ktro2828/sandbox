#include <iostream>

#include "vector.cpp"
#include "matrix.cpp"

using namespace std;

int main() {
  VectorNd<3> vec3 = {1, 2, 3};
  VectorNd<3> vec3a = {2, 2, 0};
  VectorNd<3> vec3b = {2, 3, 4};
  VectorNd<3> vec3c = {};

  Vector3d vec3d = {2, 3, 4};

  cout << "construct" << endl;
  vec3c = vec3a * vec3b;
  cout << "mul = " << vec3c << endl;
  vec3c = vec3a + vec3b;
  cout << "add = " << vec3c << endl;
  vec3c = vec3a - vec3b;
  cout << "sub = " << vec3c << endl;
  vec3c = vec3a / vec3b;
  cout << "div = " << vec3c << endl;

  double d = vec3c.dot(vec3b);
  cout << "v1 * v2 = " << d << endl;
  vec3 = vec3c.cross(vec3a);
  cout << "v1 x v2 = " << vec3 << endl;

  MatrixNd<2, 2> mat1 = {{2, 2}, {1, 1}};
  MatrixNd<2, 2> mat2 = {{2, 2}, {1, 1}};

  mat1 += mat2;
  cout << "+=mat2" << mat1 << endl;
  mat1 -= mat2;
  mat1 *= mat2;
  mat1 /= mat2;
}
