#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_timer
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/timing/Timer.h>
#include <stdio.h>
#include <unistd.h>

inline double toUSec(const double& sec)
{
  return sec * static_cast<double>(1e6);
}

inline void pause(const double& usec)
{
  fflush(stdout);
  usleep(usec);
}

namespace rt = robotics_toolkit;
namespace rtt = rt::timing;

BOOST_AUTO_TEST_CASE(test_timer)
{
  double percent_diff = 2.0; //usleep might not be so accurate

  // Time begins on construction
  rtt::Timer t;
  pause( toUSec(0.1) );
  double now = t.toc();
  BOOST_CHECK_CLOSE( now, 0.1, percent_diff );

  // Tic resets timer
  t.tic();
  pause( toUSec(0.1) );
  BOOST_CHECK_CLOSE( t.toc(), 0.1, percent_diff );

  // Ensure ptic and ptoc should print times to the console and return
  // the same times as tic and toc
  t.ptic();
  pause( toUSec(0.1) );
  BOOST_CHECK_CLOSE( t.ptoc(), 0.1, percent_diff );

  // Test multiple timer objects at once
  rtt::Timer t1;
  pause( toUSec(0.1) );
  rtt::Timer t2;
  pause( toUSec(0.1) );
  BOOST_CHECK_CLOSE( t1.toc(), 0.2, percent_diff );
  BOOST_CHECK_CLOSE( t2.toc(), 0.1, percent_diff );

  // Check that toc() doesn't drift over many calls
  t.tic();
  for (unsigned int ii = 1; ii < 100; ++ii)
  {
    pause( toUSec(0.001) );
    BOOST_CHECK_CLOSE( t.toc(), static_cast<double>(ii) * 0.001, percent_diff*15 );
  }

}
