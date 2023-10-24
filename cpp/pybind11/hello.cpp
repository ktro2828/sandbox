#include <pybind11/pybind11.h>

int add(const int i, const int j)
{
  return i + j;
}

int sub(const int i, const int j)
{
  return i - j;
}

int mul(const int i = 1, const int j = 2)
{
  return i * j;
}

struct Pet
{
  Pet(const std::string & name, const int age) : name(name), age(age) {}
  void set(const int age_) { age = age_; }
  void set(const std::string & name_) { name = name_; }
  std::string name;
  int age;
};

struct Dog : Pet
{
  Dog(const std::string & name, const int age) : Pet(name, age) {}
  std::string bark() const { return "woof!"; }
};

PYBIND11_MODULE(hello, m)  // hello: module name, m: py::module_
{
  namespace py = pybind11;

  m.doc() = "Hello, pybind11!!";

  m.def("add", &add, "A function that adds two numbers");

  // with keyword arguments
  m.def("sub", &sub, "A function that subtracts two numbers", py::arg("i"), py::arg("j"));
  // shorthand
  using namespace pybind11::literals;
  m.def("sub", &sub, "A function that subtracts two numbers", "i"_a, "j"_a);

  // default arguments
  m.def("mul", &mul, "A function that multiplies two numbers", py::arg("i") = 1, py::arg("j") = 2);

  // class
  py::class_<Pet>(m, "Pet")
    .def(py::init<const std::string &, const int>())
    .def_readwrite("name", &Pet::name)
    .def_readwrite("age", &Pet::age)
    .def("set", py::overload_cast<int>(&Pet::set), "Set the pet's age")
    .def("set", py::overload_cast<const std::string &>(&Pet::set), "Set the pet's name");

  py::class_<Dog, Pet /* <- specify C++ parent type */>(m, "Dog")
    .def(py::init<const std::string &, const int>())
    .def("bark", &Dog::bark);
}