#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_memento
#include <boost/test/unit_test.hpp>

#include <stdio.h>
#include <robotics_toolkit/design_patterns/Memento.h>

namespace rt = robotics_toolkit;
namespace dp = rt::design_patterns;

// Define Originator class which holds internal
// data that will be written to and read from
// the Memento object
class Originator
{
public:
  Originator()
  { }

  ~Originator()
  { }

  void setData(int ii_)
  {
    ii = ii_;
  }

  int getData() const
  {
    return ii;
  }

private:
  int ii;
};

BOOST_AUTO_TEST_CASE(test_memento)
{
  Originator originator;

  // Set the originator's internal data value to 5
  originator.setData(5);

  // Add scope to ensure nothing weird is happening with memory
  // when memento is deleted
  {
    // Make a memento with value 5
    dp::Memento<Originator> m("originator1");
    m.save(originator);

    // Set the originator's internal data value to 10
    originator.setData(10);

    // Two ways to re-load data
    m.reinstate(originator); // first way
    BOOST_CHECK_EQUAL(originator.getData(), 5);

    originator = m.load(); // second way
    BOOST_CHECK_EQUAL(originator.getData(), 5);
  }

  // Does anything weird happen after the memento object is destroyed?
  BOOST_CHECK_EQUAL(originator.getData(), 5);
}
