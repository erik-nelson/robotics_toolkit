#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_so3_error
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/geometry/GeometryUtils.h>

namespace rtg = robotics_toolkit::geometry;
namespace rtla = robotics_toolkit::linear_algebra;
namespace math = robotics_toolkit::math;

BOOST_AUTO_TEST_CASE(so3_matrix)
{
  rtg::Rot3 r1(0, 0, 0);
  rtg::Rot3 r2(0, 0, 0);
  rtg::Rot3 r3(M_PI/2.0, 0, 0);

  BOOST_CHECK_EQUAL( r1.error(r2), 0.0 );
  BOOST_CHECK_EQUAL( r1.error(r3), 1.0 );
}

BOOST_AUTO_TEST_CASE(so3_quat)
{
  rtg::Quat q1 = rtg::RToQuat(rtg::Rot3(0, 0, 0));
  rtg::Quat q2 = rtg::RToQuat(rtg::Rot3(0, 0, 0));
  rtg::Quat q3 = rtg::RToQuat(rtg::Rot3(M_PI/2.0, 0, 0));

  rtg::Rot3 r1(q1);
  rtg::Rot3 r2(q2);
  rtg::Rot3 r3(q3);

  rtla::Vec3 v1 = r1.row(2);
  rtla::Vec3 v2 = r2.row(2);
  rtla::Vec3 v3 = r3.row(2);

  BOOST_CHECK_EQUAL( math::acos(v1.dot(v2)), 0.0 );
  BOOST_CHECK_CLOSE( math::acos(v1.dot(v3)), M_PI/2.0 , 1e-10);

  BOOST_CHECK_CLOSE( q1.error(q2).axisAngle()(0), 0.0, 1e-10);
  BOOST_CHECK_CLOSE( q1.error(q3).axisAngle()(0), M_PI/2, 1e-10);

  BOOST_CHECK_CLOSE( math::acos(v1.dot(v2)), q1.error(q2).axisAngle()(0) , 1e-10);
  BOOST_CHECK_CLOSE( math::acos(v1.dot(v3)), q1.error(q3).axisAngle()(0) , 1e-10);
}
