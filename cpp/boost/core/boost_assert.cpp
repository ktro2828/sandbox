#include <boost/assert.hpp>

int main(int argc, char* argv[])
{
  BOOST_ASSERT(1 == 1);
  BOOST_ASSERT(1 != 1);

  // BOOST_ASSERT_MSG(argc > 1, "you must specify at least one option");
}
