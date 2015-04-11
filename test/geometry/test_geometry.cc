#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_geometry
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/geometry/GeometryUtils.h>

namespace rtg = robotics_toolkit::geometry;
namespace rtla = robotics_toolkit::linear_algebra;

BOOST_AUTO_TEST_CASE(quat_base)
{
  arma::vec4 p1(arma::fill::randu);
  arma::vec4 p2(arma::fill::randu);

  rtg::Quat q1 = rtg::Quat(p1);
  rtg::Quat q2 = rtg::Quat(p2);

  BOOST_CHECK_EQUAL(arma::norm(p1,2), q1.norm());
  BOOST_CHECK_EQUAL(rtg::Quat(p1/arma::norm(p1,2)), q1.normalize());
}

BOOST_AUTO_TEST_CASE(rot2_base)
{
  double th1 = arma::as_scalar(arma::randu());
  double th2 = arma::as_scalar(arma::randu());

  arma::mat22 p1;
  p1(0, 0) = cos(th1); p1(0, 1) = -sin(th1);
  p1(1, 0) = sin(th1); p1(1, 1) = cos(th1);

  arma::mat22 p2;
  p2(0, 0) = cos(th2); p2(0, 1) = -sin(th2);
  p2(1, 0) = sin(th2); p2(1, 1) = cos(th2);

  rtg::Rot2 m1(p1);
  rtg::Rot2 m2(p2);

  BOOST_CHECK_EQUAL(m1, rtg::Rot2(th1));
  BOOST_CHECK_EQUAL(m2, rtg::Rot2(th2));

  BOOST_CHECK_EQUAL(rtg::Rot2(p1*p2), (m1*m2));
  BOOST_CHECK_EQUAL(rtg::Rot2(p2*p1), (m2*m1));

  rtg::Rot2 r1(th1);
  rtg::Rot2 r2(th2);

  BOOST_CHECK_CLOSE(r1.error(r2), sin(th1 - th2), 1e-5);

  Eigen::Rotation2D<double> e1(m1.eigen());
  Eigen::Rotation2D<double> e2(m2.eigen());
  BOOST_CHECK_EQUAL(rtg::Rot2(e1*e2), (m1*m2));
  BOOST_CHECK_EQUAL(rtg::Rot2(e2*e1), (m2*m1));
  BOOST_CHECK_EQUAL(rtg::Rot2(Eigen::Rotation2D<double>(th1)), rtg::Rot2(th1));
}

BOOST_AUTO_TEST_CASE(rot3_base)
{
  arma::vec3 p(arma::fill::randu);
  rtg::Rot3 m1 = rtg::Rot3(rtla::Vec3(p));
  rtg::Rot3 m2 = rtg::ZYXToR(rtla::Vec3(p));
  rtg::Rot3 m3(p(0), p(1), p(2));

  BOOST_CHECK_EQUAL(m1, m2);
  BOOST_CHECK_EQUAL(m1, m3);

  BOOST_CHECK_EQUAL(m1.getEulerZYX(), rtla::Vec3(p));

  rtg::Quat q(rtg::Quat(arma::vec4(arma::fill::randu)).normalize());
  m1 = rtg::quatToR(q);
  m2 = rtg::Rot3(q);

  BOOST_CHECK_EQUAL(m1, m2);

  arma::mat33 p1 = rtla::arma(m1);
  arma::mat33 p2 = rtla::arma(m2);

  BOOST_CHECK_EQUAL(rtg::Rot3(p1*p2), (m1*m2));
  BOOST_CHECK_EQUAL(rtg::Rot3(p2*p1), (m2*m1));

  BOOST_CHECK_EQUAL(m1.getEulerZYX()(0), m1.roll());
  BOOST_CHECK_EQUAL(m1.getEulerZYX()(1), m1.pitch());
  BOOST_CHECK_EQUAL(m1.getEulerZYX()(2), m1.yaw());

  BOOST_CHECK_EQUAL(m1.getEulerZYX(), rtg::RToZYX(m1));

  Eigen::AngleAxis<double> e1(m1.eigen());
  Eigen::AngleAxis<double> e2(m2.eigen());
  BOOST_CHECK_EQUAL(rtg::Rot3(e1*e2), (m1*m2));
  BOOST_CHECK_EQUAL(rtg::Rot3(e2*e1), (m2*m1));
}

BOOST_AUTO_TEST_CASE(transform_base)
{
  rtg::Transform3 t1;
  rtg::Transform3 t2(rtla::Vec3(0, 0, 0), rtg::Rot3(0, 0, 0));
  rtg::Transform3 t3 = rtg::Transform3::identity();

  BOOST_CHECK_EQUAL(t1, t2);
  BOOST_CHECK_EQUAL(t1, t3);

  rtg::Transform3 t(rtla::Vec3(arma::vec3(arma::fill::randu)),
                    rtg::ZYXToR(rtla::Vec3(arma::vec3(arma::fill::randu))));

  BOOST_CHECK_EQUAL(rtg::pose_update(t, t1), t + t1);
}

