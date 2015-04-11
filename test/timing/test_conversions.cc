#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_conversions
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/timing/Timer.h>
#include <robotics_toolkit/timing/TimeConversions.h>
#include <stdio.h>
#include <unistd.h>

namespace rt = robotics_toolkit;
namespace rtt = rt::timing;
namespace tc = rtt::time_conversions;

inline double toUSec(const double& sec)
{
  return sec * static_cast<double>(1e6);
}

inline void pause(const double& usec)
{
  fflush(stdout);
  usleep(usec);
}

BOOST_AUTO_TEST_CASE(test_conversions)
{
  double percent_diff = 2.0; //usleep is not so accurate

  rtt::Timer t;
  t.tic();
  pause( toUSec(0.1) );
  double now = t.toc();

  BOOST_CHECK_CLOSE( now, 0.1, percent_diff );
  BOOST_CHECK_CLOSE( tc::toMSec(now), 0.1 * 1000.0, percent_diff );
  BOOST_CHECK_CLOSE( tc::toUSec(now), 0.1 * 1000000.0, percent_diff );
  BOOST_CHECK_CLOSE( tc::toNSec(now), 0.1 * 1000000000.0, percent_diff );
  BOOST_CHECK_CLOSE( tc::toMin(now), 0.1 / 60., percent_diff );
  BOOST_CHECK_CLOSE( tc::toHour(now), 0.1 / 3600., percent_diff );
  BOOST_CHECK_CLOSE( tc::toDay(now), 0.1 / 86400., percent_diff );
  BOOST_CHECK_CLOSE( tc::toYear(now), 0.1 / 31536000., percent_diff );
}
