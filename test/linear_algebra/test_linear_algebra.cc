#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_linear_algebra
#include <boost/test/unit_test.hpp>

#include <robotics_toolkit/linear_algebra/MatrixNxMBase.h>
#include <robotics_toolkit/linear_algebra/MatrixNxNBase.h>
#include <robotics_toolkit/linear_algebra/Matrix2x2.h>
#include <robotics_toolkit/linear_algebra/Matrix3x3.h>
#include <robotics_toolkit/linear_algebra/Matrix4x4.h>
#include <robotics_toolkit/linear_algebra/VectorNBase.h>
#include <robotics_toolkit/linear_algebra/Vector2.h>
#include <robotics_toolkit/linear_algebra/Vector3.h>
#include <robotics_toolkit/linear_algebra/Vector4.h>
#include <robotics_toolkit/math/MathDefinitions.h>

namespace la = robotics_toolkit::linear_algebra;

BOOST_AUTO_TEST_CASE(vector2_base)
{
  arma::vec2 p1(arma::fill::randu);
  arma::vec2 p2(arma::fill::randu);

  la::Vec2 v1(p1);
  la::Vec2 v2(p2);

  Eigen::Vector2d e1 = v1.eigen();
  Eigen::Vector2d e2 = v2.eigen();
  BOOST_CHECK_EQUAL(la::Vec2(e1 + e2), (v1 + v2));

  BOOST_CHECK_EQUAL(la::Vec2(p1 + p2), (v1 + v2));
  BOOST_CHECK_EQUAL(la::Vec2(p1 - p2), (v1 - v2));
  BOOST_CHECK_EQUAL(la::Vec2(p1 % p2), (v1 % v2));
  BOOST_CHECK_EQUAL(la::Vec2(p1 / p2), (v1 / v2));
  BOOST_CHECK_CLOSE(arma::dot(p1, p2), (v1 ^ v2), 1e-10);
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(la::Vec2(s*p1), (s*v1));
  BOOST_CHECK_EQUAL(la::Vec2(p1*s), (v1*s));
  BOOST_CHECK_CLOSE(arma::norm(p1, 2), v1.norm(), 1e-10);

  la::Vec2 v3(0.1, 0.2);
  arma::vec2 p3 = v3.arma();
  arma::vec2 p4 = la::arma(v3);
  BOOST_CHECK_EQUAL(la::Vec2(p3), la::Vec2(p4));
  BOOST_CHECK_EQUAL(la::Vec2(s*p3), (s*v3));
  BOOST_CHECK_EQUAL(la::Vec2(p1/arma::norm(p1,2)), v1.normalize());

  v3 /= s;
  p3 /= s;
  BOOST_CHECK_EQUAL(v3, la::Vec2(p3));

  v3 += v2;
  p3 += p2;
  BOOST_CHECK_EQUAL(v3, la::Vec2(p3));

  v3 -= v2;
  p3 -= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec2(p3));

  v3 %= v2;
  p3 %= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec2(p3));

  v3 /= v2;
  p3 /= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec2(p3));
}

BOOST_AUTO_TEST_CASE(vector3_base)
{
  arma::vec3 p1(arma::fill::randu);
  arma::vec3 p2(arma::fill::randu);

  la::Vec3 v1(p1);
  la::Vec3 v2(p2);

  Eigen::Vector3d e1 = v1.eigen();
  Eigen::Vector3d e2 = v2.eigen();
  BOOST_CHECK_EQUAL(la::Vec3(e1 + e2), (v1 + v2));

  BOOST_CHECK_EQUAL(la::Vec3(p1 + p2), (v1 + v2));
  BOOST_CHECK_EQUAL(la::Vec3(p1 - p2), (v1 - v2));
  BOOST_CHECK_EQUAL(la::Vec3(p1 % p2), (v1 % v2));
  BOOST_CHECK_EQUAL(la::Vec3(p1 / p2), (v1 / v2));
  BOOST_CHECK_CLOSE(arma::dot(p1, p2), (v1 ^ v2), 1e-10);
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(la::Vec3(s*p1), (s*v1));
  BOOST_CHECK_EQUAL(la::Vec3(p1*s), (v1*s));
  BOOST_CHECK_CLOSE(arma::norm(p1, 2), v1.norm(), 1e-10);

  BOOST_CHECK_EQUAL(la::Vec3(arma::cross(p1, p2)), v1.cross(v2));
  BOOST_CHECK_EQUAL(la::Vec3(arma::cross(p1, p2)), cross(v1, v2));

  la::Vec3 v3(0.1, 0.2, 0.3);
  arma::vec3 p3 = v3.arma();
  arma::vec3 p4 = la::arma(v3);
  BOOST_CHECK_EQUAL(la::Vec3(p3), la::Vec3(p4));
  BOOST_CHECK_EQUAL(la::Vec3(s*p3), (s*v3));

  BOOST_CHECK_EQUAL(la::Vec3(p1/arma::norm(p1,2)), v1.normalize());

  v3 /= s;
  p3 /= s;
  BOOST_CHECK_EQUAL(v3, la::Vec3(p3));

  v3 += v2;
  p3 += p2;
  BOOST_CHECK_EQUAL(v3, la::Vec3(p3));

  v3 -= v2;
  p3 -= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec3(p3));

  v3 %= v2;
  p3 %= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec3(p3));

  v3 /= v2;
  p3 /= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec3(p3));
}

BOOST_AUTO_TEST_CASE(vector4_base)
{
  arma::vec4 p1(arma::fill::randu);
  arma::vec4 p2(arma::fill::randu);

  la::Vec4 v1(p1);
  la::Vec4 v2(p2);

  Eigen::Vector4d e1 = v1.eigen();
  Eigen::Vector4d e2 = v2.eigen();
  BOOST_CHECK_EQUAL(la::Vec4(e1 + e2), (v1 + v2));

  BOOST_CHECK_EQUAL(la::Vec4(p1 + p2), (v1 + v2));
  BOOST_CHECK_EQUAL(la::Vec4(p1 - p2), (v1 - v2));
  BOOST_CHECK_EQUAL(la::Vec4(p1 % p2), (v1 % v2));
  BOOST_CHECK_EQUAL(la::Vec4(p1 / p2), (v1 / v2));
  BOOST_CHECK_CLOSE(arma::dot(p1, p2), (v1 ^ v2), 1e-10);
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(la::Vec4(s*p1), (s*v1));
  BOOST_CHECK_EQUAL(la::Vec4(p1*s), (v1*s));
  BOOST_CHECK_CLOSE(arma::norm(p1, 2), v1.norm(), 1e-10);

  la::Vector4 v3(0.1, 0.2, 0.3, 0.4);
  arma::vec4 p3 = v3.arma();
  arma::vec4 p4 = la::arma(v3);
  BOOST_CHECK_EQUAL(la::Vec4(p3), la::Vec4(p4));
  BOOST_CHECK_EQUAL(la::Vec4(s*p3), (s*v3));

  BOOST_CHECK_EQUAL(la::Vec4(p1/arma::norm(p1,2)), v1.normalize());

  v3 /= s;
  p3 /= s;
  BOOST_CHECK_EQUAL(v3, la::Vec4(p3));

  v3 += v2;
  p3 += p2;
  BOOST_CHECK_EQUAL(v3, la::Vec4(p3));

  v3 -= v2;
  p3 -= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec4(p3));

  v3 %= v2;
  p3 %= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec4(p3));

  v3 /= v2;
  p3 /= p2;
  BOOST_CHECK_EQUAL(v3, la::Vec4(p3));
}

// BOOST_AUTO_TEST_CASE(quat_base)
// {
//   arma::vec4 p1(arma::fill::randu);
//   arma::vec4 p2(arma::fill::randu);

//   la::Quat q1 = la::Quat(p1);
//   la::Quat q2 = la::Quat(p2);

//   BOOST_CHECK_EQUAL(arma::norm(p1,2), q1.norm());
//   BOOST_CHECK_EQUAL(la::Quat(p1/arma::norm(p1,2)), q1.normalize());
// }

BOOST_AUTO_TEST_CASE(matrix2x2_base)
{
  arma::mat22 p1(arma::fill::randu);
  arma::mat22 p2(arma::fill::randu);
  arma::vec2 pv(arma::fill::randu);

  la::Mat22 m1(p1);
  la::Mat22 m2(p2);
  la::Vec2 v(pv);

  Eigen::Matrix2d e1 = m1.eigen();
  Eigen::Matrix2d e2 = m2.eigen();
  BOOST_CHECK_EQUAL(la::Mat22(e1 + e2), (m1 + m2));

  BOOST_CHECK_EQUAL(la::Mat22(p1 + p2), (m1 + m2));
  BOOST_CHECK_EQUAL(la::Mat22(p1 - p2), (m1 - m2));
  BOOST_CHECK_EQUAL(la::Mat22(p1 % p2), (m1 % m2));
  BOOST_CHECK_EQUAL(la::Mat22(p1 / p2), (m1 / m2));
  BOOST_CHECK_EQUAL(la::Mat22(p1 * p2), (m1 * m2));
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(la::Mat22(s*p1), (s*m1));
  BOOST_CHECK_EQUAL(la::Mat22(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(arma::norm(p1, "fro"), m1.norm(), 1e-10);
  BOOST_CHECK_EQUAL(la::Vec2(p1*pv), (m1*v));

  la::Mat22 v3(0.1, 0.2, 0.3, 0.4);
  arma::mat22 p3 = v3.arma();
  arma::mat22 p4 = la::arma(v3);
  BOOST_CHECK_EQUAL(la::Mat22(p3), la::Mat22(p4));
  BOOST_CHECK_EQUAL(la::Mat22(s*p3), (s*v3));

  m1 += m2;
  p1 += p2;
  BOOST_CHECK_EQUAL(la::Mat22(p1), m1);

  m1 -= m2;
  p1 -= p2;
  BOOST_CHECK_EQUAL(la::Mat22(p1), m1);

  la::Vec2 sing(m1.singularValues());
  arma::mat22 U;
  arma::vec2 si;
  arma::mat22 V;
  arma::svd(U,si,V,p1);

  BOOST_CHECK_EQUAL(la::Vec2(si), sing);

  la::Mat22 gmsing(1, 3, 2, 6);
  arma::mat22 amsing = gmsing.arma();

  try
  {
    la::Mat22 tmp = gmsing.inv();
    tmp.print("Problem if printed");
  }
  catch (const std::exception& e)
  {
    std::cerr << "inv of singular matrix failed as expected" << std::endl;
  }

  try
  {
    arma::mat22 tmp = arma::inv(amsing);
    tmp.print("Problem if printed");
  }
  catch (const std::exception& e)
  {
    std::cerr << "arma inv of singular matrix failed as expected:" << std::endl;
  }

  BOOST_CHECK_EQUAL(la::Mat22(arma::inv(p1)), la::inv(m1));
  BOOST_CHECK_CLOSE(arma::det(p1), m1.det(), 1e-10);

  arma::vec2 ov1(arma::fill::randu);
  arma::vec2 ov2(arma::fill::randu);
  la::Vec2 gov1(ov1), gov2(ov2);
  BOOST_CHECK_EQUAL(la::Mat22(ov1*arma::trans(ov2)), la::outer(gov1, gov2));
}

BOOST_AUTO_TEST_CASE(matrix3x3_base)
{
  arma::mat33 p1(arma::fill::randu);
  arma::mat33 p2(arma::fill::randu);
  arma::vec3 pv(arma::fill::randu);

  la::Mat33 m1(p1);
  la::Mat33 m2(p2);
  la::Vec3 v(pv);

  BOOST_CHECK_EQUAL(m1, m1);
  BOOST_CHECK(m1 != m2);

  Eigen::Matrix3d e1 = m1.eigen();
  Eigen::Matrix3d e2 = m2.eigen();
  BOOST_CHECK_EQUAL(la::Mat33(e1 + e2), (m1 + m2));

  BOOST_CHECK_EQUAL(la::Mat33(p1 + p2), (m1 + m2));
  BOOST_CHECK_EQUAL(la::Mat33(p1 - p2), (m1 - m2));
  BOOST_CHECK_EQUAL(la::Mat33(p1 % p2), (m1 % m2));
  BOOST_CHECK_EQUAL(la::Mat33(p1 / p2), (m1 / m2));
  BOOST_CHECK_EQUAL(la::Mat33(p1 * p2), (m1 * m2));
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(la::Mat33(s*p1), (s*m1));
  BOOST_CHECK_EQUAL(la::Mat33(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(arma::norm(p1, "fro"), m1.norm(), 1e-10);
  BOOST_CHECK_EQUAL(la::Vec3(p1*pv), (m1*v));

  la::Mat33 v3(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9);
  arma::mat33 p3 = v3.arma();
  arma::mat33 p4 = la::arma(v3);
  BOOST_CHECK_EQUAL(la::Mat33(p3), la::Mat33(p4));
  BOOST_CHECK_EQUAL(la::Mat33(s*p3), (s*v3));

  m1 += m2;
  p1 += p2;
  BOOST_CHECK_EQUAL(la::Mat33(p1), m1);

  m1 -= m2;
  p1 -= p2;
  BOOST_CHECK_EQUAL(la::Mat33(p1), m1);

  BOOST_CHECK_EQUAL(la::Mat33(arma::inv(p1)), la::inv(m1));
  BOOST_CHECK_CLOSE(arma::det(p1), m1.det(), 1e-10);

  arma::vec3 ov1(arma::fill::randu);
  arma::vec3 ov2(arma::fill::randu);
  la::Vec3 gov1(ov1), gov2(ov2);
  BOOST_CHECK_EQUAL(la::Mat33(ov1*arma::trans(ov2)), la::outer(gov1, gov2));
}

BOOST_AUTO_TEST_CASE(matrix4x4_base)
{
  arma::mat44 p1(arma::fill::randu);
  arma::mat44 p2(arma::fill::randu);
  arma::vec4 pv(arma::fill::randu);

  la::Mat44 m1(p1);
  la::Mat44 m2(p2);
  la::Vec4 v(pv);

  BOOST_CHECK_EQUAL(m1, m1);
  BOOST_CHECK(m1 != m2);

  Eigen::Matrix4d e1 = m1.eigen();
  Eigen::Matrix4d e2 = m2.eigen();
  BOOST_CHECK_EQUAL(la::Mat44(e1 + e2), (m1 + m2));

  BOOST_CHECK_EQUAL(la::Mat44(p1 + p2), (m1 + m2));
  BOOST_CHECK_EQUAL(la::Mat44(p1 - p2), (m1 - m2));
  BOOST_CHECK_EQUAL(la::Mat44(p1 % p2), (m1 % m2));
  BOOST_CHECK_EQUAL(la::Mat44(p1 / p2), (m1 / m2));
  BOOST_CHECK_EQUAL(la::Mat44(p1 * p2), (m1 * m2));
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(la::Mat44(s*p1), (s*m1));
  BOOST_CHECK_EQUAL(la::Mat44(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(arma::norm(p1, "fro"), m1.norm(), 1e-10);
  BOOST_CHECK_EQUAL(la::Vec4(p1*pv), (m1*v));

  la::Mat44 v3(0.1, 0.2, 0.3, 0.4,
               0.5, 0.6, 0.7, 0.8,
               0.9, 1.0, 1.1, 1.2,
               1.3, 1.4, 1.5, 1.6);
  arma::mat44 p3 = v3.arma();
  arma::mat44 p4 = la::arma(v3);
  BOOST_CHECK_EQUAL(la::Mat44(p3), la::Mat44(p4));
  BOOST_CHECK_EQUAL(la::Mat44(s*p3), (s*v3));

  m1 += m2;
  p1 += p2;
  BOOST_CHECK_EQUAL(la::Mat44(p1), m1);

  m1 -= m2;
  p1 -= p2;
  BOOST_CHECK_EQUAL(la::Mat44(p1), m1);

  BOOST_CHECK_EQUAL(la::Mat44(arma::inv(p1)), la::inv(m1));
  BOOST_CHECK_CLOSE(arma::det(p1), m1.det(), 1e-10);

  arma::vec4 ov1(arma::fill::randu);
  arma::vec4 ov2(arma::fill::randu);
  la::Vec4 gov1(ov1), gov2(ov2);
  BOOST_CHECK_EQUAL(la::Mat44(ov1*arma::trans(ov2)), la::outer(gov1, gov2));
}

// BOOST_AUTO_TEST_CASE(rot2_base)
// {
//   double th1 = arma::as_scalar(arma::randu());
//   double th2 = arma::as_scalar(arma::randu());

//   arma::mat22 p1;
//   p1(0, 0) = cos(th1); p1(0, 1) = -sin(th1);
//   p1(1, 0) = sin(th1); p1(1, 1) = cos(th1);

//   arma::mat22 p2;
//   p2(0, 0) = cos(th2); p2(0, 1) = -sin(th2);
//   p2(1, 0) = sin(th2); p2(1, 1) = cos(th2);

//   la::Rot2 m1(p1);
//   la::Rot2 m2(p2);

//   BOOST_CHECK_EQUAL(m1, la::Rot2(th1));
//   BOOST_CHECK_EQUAL(m2, la::Rot2(th2));

//   BOOST_CHECK_EQUAL(la::Rot2(p1*p2), (m1*m2));
//   BOOST_CHECK_EQUAL(la::Rot2(p2*p1), (m2*m1));

//   la::Rot2 r1(th1);
//   la::Rot2 r2(th2);

//   BOOST_CHECK_EQUAL(r1.error(r2), sin(th1 - th2));

//   Eigen::Rotation2D<double> e1(m1.eigen());
//   Eigen::Rotation2D<double> e2(m2.eigen());
//   BOOST_CHECK_EQUAL(la::Rot2(e1*e2), (m1*m2));
//   BOOST_CHECK_EQUAL(la::Rot2(e2*e1), (m2*m1));
//   BOOST_CHECK_EQUAL(la::Rot2(Eigen::Rotation2D<double>(th1)), la::Rot2(th1));
// }

// BOOST_AUTO_TEST_CASE(rot3_base)
// {
//   arma::vec3 p(arma::fill::randu);
//   la::Rot3 m1 = la::Rot3(la::Vec3(p));
//   la::Rot3 m2 = la::ZYXToR(la::Vec3(p));
//   la::Rot3 m3(p(0), p(1), p(2));

//   BOOST_CHECK_EQUAL(m1, m2);
//   BOOST_CHECK_EQUAL(m1, m3);

//   BOOST_CHECK_EQUAL(m1.getEulerZYX(), la::Vec3(p));

//   la::Quat q(la::Quat(arma::vec4(arma::fill::randu)).normalize());
//   m1 = la::QuatToR(q);
//   m2 = la::Rot3(q);

//   BOOST_CHECK_EQUAL(m1, m2);

//   arma::mat33 p1 = la::arma(m1);
//   arma::mat33 p2 = la::arma(m2);

//   BOOST_CHECK_EQUAL(la::Rot3(p1*p2), (m1*m2));
//   BOOST_CHECK_EQUAL(la::Rot3(p2*p1), (m2*m1));

//   BOOST_CHECK_EQUAL(m1.getEulerZYX()(0), m1.roll());
//   BOOST_CHECK_EQUAL(m1.getEulerZYX()(1), m1.pitch());
//   BOOST_CHECK_EQUAL(m1.getEulerZYX()(2), m1.yaw());

//   BOOST_CHECK_EQUAL(m1.getEulerZYX(), la::RToZYX(m1));

//   Eigen::AngleAxis<double> e1(m1.eigen());
//   Eigen::AngleAxis<double> e2(m2.eigen());
//   BOOST_CHECK_EQUAL(la::Rot3(e1*e2), (m1*m2));
//   BOOST_CHECK_EQUAL(la::Rot3(e2*e1), (m2*m1));
// }

// BOOST_AUTO_TEST_CASE(transform_base)
// {
//   la::Transform3 t1;
//   la::Transform3 t2(la::Vec3(0, 0, 0), la::Rot3(0, 0, 0));
//   la::Transform3 t3 = la::Transform3::identity();

//   BOOST_CHECK_EQUAL(t1, t2);
//   BOOST_CHECK_EQUAL(t1, t3);

//   la::Transform3 t(la::Vec3(arma::vec3(arma::fill::randu)),
//                    la::ZYXToR(la::Vec3(arma::vec3(arma::fill::randu))));

//   BOOST_CHECK_EQUAL(la::pose_update(t, t1), t + t1);
// }

BOOST_AUTO_TEST_CASE(matrixnxm_base)
{
  typedef arma::mat::fixed<4, 2> am42;
  typedef arma::mat::fixed<2, 4> am24;
  typedef arma::mat::fixed<4, 4> am44;
  typedef arma::vec::fixed<2> av2;
  typedef arma::vec::fixed<4> av4;

  typedef la::MatrixNxMBase<double, 4, 2> gm42;
  typedef la::MatrixNxMBase<double, 2, 4> gm24;
  typedef la::MatrixNxMBase<double, 4, 4> gm44;
  typedef la::VectorNBase<double, 2> gv2;
  typedef la::VectorNBase<double, 4> gv4;

  am42 p1(arma::fill::randu);
  am24 p2(arma::fill::randu);
  am44 p3(arma::fill::randu);
  am24 p4(arma::fill::randu);
  av2 pv1(arma::fill::randu);
  av4 pv2(arma::fill::randu);

  gm42 m1(p1);
  gm24 m2(p2);
  gm44 m3(p3);
  gm24 m4(p4);
  gv2 v1(pv1);
  gv4 v2(pv2);

  BOOST_CHECK_EQUAL(la::Vec4(p1*pv1), (m1*v1));
  BOOST_CHECK_EQUAL(la::Vec2(p2*pv2), (m2*v2));
  BOOST_CHECK_EQUAL(la::Mat44(p1*p2), (m1*m2));
  BOOST_CHECK_EQUAL(la::Mat22(p2*p1), (m2*m1));

  BOOST_CHECK_EQUAL(m1, m1);
  BOOST_CHECK_EQUAL(m2, m2);
  BOOST_CHECK_EQUAL(m3, m3);
  BOOST_CHECK(m4 != m2);

  BOOST_CHECK_EQUAL(gm42(p1 + p1), (m1 + m1));
  BOOST_CHECK_EQUAL(gm24(p4 - p2), (m4 - m2));
  BOOST_CHECK_EQUAL(gm42(p1 % p1), (m1 % m1));
  BOOST_CHECK_EQUAL(gm24(p4 / p2), (m4 / m2));
  BOOST_CHECK_EQUAL(gm44(p1 * p2), (m1 * m2));
  double s = arma::as_scalar(arma::randu());
  BOOST_CHECK_EQUAL(gm24(s*p2), (s*m2));
  BOOST_CHECK_EQUAL(gm42(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(arma::norm(p3, "fro"), m3.norm(), 1e-10);
  BOOST_CHECK_CLOSE(arma::norm(p1*p2, "fro"), (m1*m2).norm(), 1e-10);

  gm24 m5;
  m5(0, 0) = 0.1; m5(0, 1) = 0.2; m5(0, 2) = 0.2; m5(0, 3) = 0.3;
  m5(1, 0) = 1.1; m5(1, 1) = 1.2; m5(1, 2) = 1.2; m5(1, 3) = 1.3;

  am24 p5 = m5.arma();
  am24 p6 = la::arma(m5);
  BOOST_CHECK_EQUAL(gm24(p5), gm24(p6));
  BOOST_CHECK_EQUAL(gm24(s*p5), (s*m5));

  m1 += m1;
  p1 += p1;
  BOOST_CHECK_EQUAL(gm42(p1), m1);

  m2 -= m4;
  p2 -= p4;
  BOOST_CHECK_EQUAL(gm24(p2), m2);

  av4 ov1(arma::fill::randu);
  av2 ov2(arma::fill::randu);
  gv4 gov1(ov1);
  gv2 gov2(ov2);
  BOOST_CHECK_EQUAL(gm42(ov1*arma::trans(ov2)), la::outer(gov1, gov2));
}
