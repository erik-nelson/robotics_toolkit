#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_stopwatch
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/timing/Stopwatch.h>
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

BOOST_AUTO_TEST_CASE(test_stopwatch)
{
  rtt::Stopwatch s;
  double now = s.tic();
  double elapsed = s.toc();

  s.stopwatchReset();
  for (unsigned int ii = 0; ii < 300; ++ii)
  {
    pause( toUSec(0.0005) );

    s.start();
    pause( toUSec(0.001) );
    s.stop();
  }

  BOOST_CHECK_CLOSE(s.stopwatchElapsed(), 0.001*300., 15.0);

  // Run the stopwatch twice in a row
  s.stopwatchReset();
  for (unsigned int ii = 0; ii < 300; ++ii)
  {
    pause( toUSec(0.0005) );

    s.start();
    pause( toUSec(0.001) );
    s.stop();
  }

  BOOST_CHECK_CLOSE(s.stopwatchElapsed(), 0.001*300., 15.0);

  // Run the stopwatch twice without restarting in between
  for (unsigned int ii = 0; ii < 300; ++ii)
  {
    pause( toUSec(0.0005) );

    s.start();
    pause( toUSec(0.001) );
    s.stop();
  }

  BOOST_CHECK_CLOSE(s.stopwatchElapsed(), 0.001*600., 15.0);

  printf("TEST: Four warning messages should be printed below\n\n");

  // Interrupt the stop watch with tic() calls
  s.stopwatchReset();
  s.start();
  pause( toUSec(0.01) );
  s.tic();
  pause( toUSec(0.01) );
  BOOST_CHECK_CLOSE(s.stop(), 0.01, 2.0);

  s.stopwatchOff();
  s.start();
  s.stop();

  s.stopwatchReset();
  s.stop();
  s.start();
  s.stop();
}

