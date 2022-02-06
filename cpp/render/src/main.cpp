#include <iostream>

#include "vector.hpp"

using namespace std;

int main() {
  VectorNd<3> vec3 = {1, 2, 3};
  VectorNd<3> vec3a = {2, 2, 0};
  VectorNd<3> vec3b = {2, 3, 4};
  VectorNd<3> vec3c = {};

  cout << "construct" << endl;
  vec3c = vec3a * vec3b;
  cout << "mul" << endl;
  vec3c = vec3a + vec3b;
  cout << "add" << endl;
  vec3c = vec3a - vec3b;
  cout << "sub" << endl;
  vec3c = vec3a / vec3b;
  cout << "div" << endl;
}
