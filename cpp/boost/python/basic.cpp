#include <boost/python.hpp>

// === C++ definition ===
int add(int x, int y)
{
  return x + y;
}

class Accumulator
{
private:
  int value_;

public:
  int operator()(int v)
  {
    value_ += v;
    return value_;
  }

  int value() const { return value_; }
};  // class Accumulator

// === Python object ===
BOOST_PYTHON_MODULE(basic)
{
  boost::python::def("add", add);

  boost::python::class_<Accumulator>("Accumulator")
    .def("__call__", &Accumulator::operator())
    .add_property("value", &Accumulator::value);
}