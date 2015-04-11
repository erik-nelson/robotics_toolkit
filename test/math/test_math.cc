#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_math
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/math/MathDefinitions.h>

namespace rtm = robotics_toolkit::math;

BOOST_AUTO_TEST_CASE(math)
{
  srand(time(NULL));
  float rf = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
  double rd = static_cast<double>(rand())/static_cast<double>(RAND_MAX);

  BOOST_CHECK_EQUAL(rtm::cos<float>(rf), cosf(rf));
  BOOST_CHECK_EQUAL(rtm::cos<float>(rf), rtm::cos(rf));
  BOOST_CHECK_EQUAL(rtm::cos<double>(rd), cos(rd));
  BOOST_CHECK_EQUAL(rtm::cos<double>(rd), rtm::cos(rd));

  BOOST_CHECK_EQUAL(rtm::acos<float>(rf), acosf(rf));
  BOOST_CHECK_EQUAL(rtm::acos<float>(rf), rtm::acos(rf));
  BOOST_CHECK_EQUAL(rtm::acos<double>(rd), acos(rd));
  BOOST_CHECK_EQUAL(rtm::acos<double>(rd), rtm::acos(rd));

  BOOST_CHECK_EQUAL(rtm::sin<float>(rf), sinf(rf));
  BOOST_CHECK_EQUAL(rtm::sin<float>(rf), rtm::sin(rf));
  BOOST_CHECK_EQUAL(rtm::sin<double>(rd), sin(rd));
  BOOST_CHECK_EQUAL(rtm::sin<double>(rd), rtm::sin(rd));

  BOOST_CHECK_EQUAL(rtm::asin<float>(rf), asinf(rf));
  BOOST_CHECK_EQUAL(rtm::asin<float>(rf), rtm::asin(rf));
  BOOST_CHECK_EQUAL(rtm::asin<double>(rd), asin(rd));
  BOOST_CHECK_EQUAL(rtm::asin<double>(rd), rtm::asin(rd));

  BOOST_CHECK_EQUAL(rtm::tan<float>(rf), tanf(rf));
  BOOST_CHECK_EQUAL(rtm::tan<float>(rf), rtm::tan(rf));
  BOOST_CHECK_EQUAL(rtm::tan<double>(rd), tan(rd));
  BOOST_CHECK_EQUAL(rtm::tan<double>(rd), rtm::tan(rd));

  BOOST_CHECK_EQUAL(rtm::fabs<float>(rf), fabsf(rf));
  BOOST_CHECK_EQUAL(rtm::fabs<float>(rf), rtm::fabs(rf));
  BOOST_CHECK_EQUAL(rtm::fabs<double>(rd), fabs(rd));
  BOOST_CHECK_EQUAL(rtm::fabs<double>(rd), rtm::fabs(rd));

  BOOST_CHECK_EQUAL(rtm::sqrt<float>(rf), sqrtf(rf));
  BOOST_CHECK_EQUAL(rtm::sqrt<float>(rf), rtm::sqrt(rf));
  BOOST_CHECK_EQUAL(rtm::sqrt<double>(rd), sqrt(rd));
  BOOST_CHECK_EQUAL(rtm::sqrt<double>(rd), rtm::sqrt(rd));

  float rf1 = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
  float rf2 = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
  double rd1 = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
  double rd2 = static_cast<double>(rand())/static_cast<double>(RAND_MAX);

  BOOST_CHECK_EQUAL(rtm::atan2<float>(rf1, rf2), atan2f(rf1, rf2));
  BOOST_CHECK_EQUAL(rtm::atan2<float>(rf1, rf2), rtm::atan2(rf1, rf2));
  BOOST_CHECK_EQUAL(rtm::atan2<double>(rd1, rd2), atan2(rd1, rd2));
  BOOST_CHECK_EQUAL(rtm::atan2<double>(rd1, rd2), rtm::atan2(rd1, rd2));

  BOOST_CHECK_EQUAL(rtm::hypot<float>(rf1, rf2), hypotf(rf1, rf2));
  BOOST_CHECK_EQUAL(rtm::hypot<float>(rf1, rf2), rtm::hypot(rf1, rf2));
  BOOST_CHECK_EQUAL(rtm::hypot<double>(rd1, rd2), hypot(rd1, rd2));
  BOOST_CHECK_EQUAL(rtm::hypot<double>(rd1, rd2), rtm::hypot(rd1, rd2));
}
