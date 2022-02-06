#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "stl.h"

namespace stl
{
  std::ostream& operator<<(std::ostream& out, const Point p) {
    out << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    return out;
  }

  std::ostream& operator<<(std::ostream& out, const Triangle& t) {
    out << "--- TRIANGLE ---" << std::endl;
    out << t.normal << std::endl;
    out << t.v1 << std::endl;
    out << t.v2 << std::endl;
    out << t.v3 << std::endl;

    return out;
  }

  float parse_float(std::ifstream& s) {
    char f_buf[sizeof(float)];
    s.read(f_buf, 4);
    float* fptr = (float*)f_buf;
    return *fptr;
  }

  Point parse_point(std::ifstream& s) {
    float x = parse_float(s);
    float y = parse_float(s);
    float z = parse_float(s);
    return Point(x, y, z);
  }

  STL loadSTL(const std::string& filename) {
    std::ifstream stl_file(filename.c_str(), std::ios::in | std::ios::binary);
    if (!stl_file) {
      std::cerr << "[ERROR]: cannot read file: " << filename << std::endl;
      assert(false);
    }

    char header_info[80] = "";
    char n_triangles[4];
    stl_file.read(header_info, 80);
    stl_file.read(n_triangles, 4);
    std::string h(header_info);
    STL info(h);
    unsigned int* r = (unsigned int*)n_triangles;
    unsigned int num_triangles = *r;
    for (unsigned int i=0; i<num_triangles; i++) {
      auto normal = parse_point(stl_file);
      auto v1 = parse_point(stl_file);
      auto v2 = parse_point(stl_file);
      auto v3 = parse_point(stl_file);
      info.triangles.push_back(Triangle(normal, v1, v2, v3));
      char dummy[2];
      stl_file.read(dummy, 2);
    }

    return info;
  }

} // namespace stl
