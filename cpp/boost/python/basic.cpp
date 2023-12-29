#include <boost/python.hpp>

#include <string>

int add(int x, int y)
{
  return x + y;
}

class Accumulator
{
private:
  int value_;

public:
  int operator()(const int v)
  {
    value_ += v;
    return value_;
  }

  int value() const { return value_; }
};  // class Accumulator

enum Color { RED = 1, GREEN = 2, BLUE = 3 };

// NOTE: Module name must be same with built library name in CMakeLists.txt
BOOST_PYTHON_MODULE(sample)
{
  namespace bp = boost::python;
  // whether to show all docstring with object.__doc__
  bp::docstring_options doc_options(true);

  bp::def("add", add, bp::args("x", "y"), "Performs the `+` operation.");

  bp::class_<Accumulator>("Accumulator")
    .def("__call__", &Accumulator::operator())
    .add_property("value", &Accumulator::value);

  bp::enum_<Color>("Color").value("RED", RED).value("GREEN", GREEN).value("BLUE", BLUE);
}