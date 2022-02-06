#include <cassert>
#include <iostream>

#include "stl.h"

int main(int argc, char** argv) {
  std::string filename = "./Box1x1x1.stl";

  if (argc == 2) {
    filename = argv[1];
  } else if (argc > 2) {
    std::cerr << "[ERROR]: Too many command line arguments" << std::endl;
  }

  stl::STL stl = stl::loadSTL(filename);

  std::vector<stl::Triangle> triangles = stl.triangles;
  std::cout << "STL Header = " << stl.name << std::endl;
  std::cout << "# triangles = " << triangles.size() << std::endl;
  for (size_t i=0; i<stl.triangles.size(); ++i) {
    std::cout << stl.triangles[i] << std::endl;
  }
}
