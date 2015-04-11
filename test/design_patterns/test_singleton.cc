#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_singleton
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/design_patterns/Singleton.h>

namespace rt = robotics_toolkit;
namespace dp = rt::design_patterns;

class Foo
{
public:

  Foo()
  { }

  ~Foo()
  { }

  void setData(int data_)
  {
    data = data_;
  }

  int getData() const
  {
    return data;
  }

private:
  int data;
};

BOOST_AUTO_TEST_CASE(test_singleton)
{
  // Get a singleton of the Foo object
  dp::Singleton<Foo>::instance().setData(50);
  BOOST_CHECK_EQUAL(dp::Singleton<Foo>::instance().getData(), 50);

  // Make a Foo object that is not a singleton
  Foo s1;
  s1.setData(100);
  BOOST_CHECK_EQUAL(s1.getData(), 100);

  // Ensure that the singleton Foo is independently of non-singleton Foo
  BOOST_CHECK_EQUAL(dp::Singleton<Foo>::instance().getData(), 50);

  // Make sure that modifying the singleton Foo does not modify the non-singleton Foo
  dp::Singleton<Foo>::instance().setData(150);
  BOOST_CHECK_EQUAL(dp::Singleton<Foo>::instance().getData(), 150);
  BOOST_CHECK_EQUAL(s1.getData(), 100);

  // Make sure reference to singleton works
  Foo& s = dp::Singleton<Foo>::instance();
  s.setData(10);

  BOOST_CHECK_EQUAL(s.getData(), 10);
  BOOST_CHECK_EQUAL(s.getData(), dp::Singleton<Foo>::instance().getData());
}
