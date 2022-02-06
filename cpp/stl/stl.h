#ifndef STL_H_
#define STL_H_

#include <string>
#include <vector>

namespace stl
{
  struct Point
  {
    float x;
    float y;
    float z;

  Point() : x(0), y(0), z(0) {}
  Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  }; // struct Point

  struct Triangle
  {
    Point normal;
    Point v1;
    Point v2;
    Point v3;
  Triangle(Point normal_, Point v1_, Point v2_, Point v3_) :
    normal(normal_), v1(v1_), v2(v2_), v3(v3_) {}
  }; // struct Triangle

  std::ostream& operator<<(std::ostream& out, const Triangle& t);

  struct STL
  {
    std::string name;
    std::vector<Triangle> triangles;
  STL(std::string name_) : name(name_) {}
  }; // struct STL

  STL loadSTL(const std::string& filename);

}
#endif // STL_H_
