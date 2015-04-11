/*
  geometry_utils: Utility library to provide common geometry types and transformations

  Copyright (C) 2015  Erik Nelson
  Copyright (C) 2014  Nathan Michael

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef __ROBOTICS_TOOLKIT_GEOMETRY_UTILS_H__
#define __ROBOTICS_TOOLKIT_GEOMETRY_UTILS_H__

#include <robotics_toolkit/linear_algebra/VectorNBase.h>
#include <robotics_toolkit/linear_algebra/Vector2.h>
#include <robotics_toolkit/linear_algebra/Vector3.h>
#include <robotics_toolkit/linear_algebra/Vector4.h>
#include <robotics_toolkit/linear_algebra/MatrixNxMBase.h>
#include <robotics_toolkit/linear_algebra/MatrixNxNBase.h>
#include <robotics_toolkit/linear_algebra/Matrix2x2.h>
#include <robotics_toolkit/linear_algebra/Matrix3x3.h>
#include <robotics_toolkit/linear_algebra/Matrix4x4.h>
#include <robotics_toolkit/geometry/Quaternion.h>
#include <robotics_toolkit/geometry/Rotation2.h>
#include <robotics_toolkit/geometry/Rotation3.h>
#include <robotics_toolkit/geometry/Transform2.h>
#include <robotics_toolkit/geometry/Transform3.h>

namespace robotics_toolkit
{
  namespace geometry
  {

    namespace la = ::robotics_toolkit::linear_algebra;

    inline double unroll(double x)
    {
      x = fmod(x, 2.0*M_PI);
      if (x < 0) x += 2.0*M_PI;
      return x;
    }

    inline double normalize(double x)
    {
      x = fmod(x + M_PI, 2.0*M_PI);
      if (x < 0) x += 2.0*M_PI;
      return x - M_PI;
    }

    inline double shortestAngularDistance(double from, double to)
    {
      double result = unroll(unroll(to) - unroll(from));
      if (result > M_PI)
        result = -(2.0*M_PI - result);
      return normalize(result);
    }

    inline double radToDeg(double angle)
    {
      return angle*180.0*M_1_PI;
    }

    inline double degToRad(double angle)
    {
      return angle*M_PI/180.0;
    }

    inline la::Vec3 radToDeg(const la::Vec3& angles)
    {
      return la::Vec3(radToDeg(angles(0)),
                      radToDeg(angles(1)),
                      radToDeg(angles(2)));
    }

    inline la::Vec3 degToRad(const la::Vec3& angles)
    {
      return la::Vec3(degToRad(angles(0)),
                      degToRad(angles(1)),
                      degToRad(angles(2)));
    }

    inline la::Vec3 RToZYX(const Rot3& rot)
    {
      return rot.getEulerZYX();
    }

    inline Rot3 ZYXToR(const la::Vec3& angles)
    {
      return Rot3(angles);
    }

    inline Rot3 quatToR(const Quat& quat)
    {
      return Rot3(quat);
    }

    inline Quat RToQuat(const Rot3& rot)
    {
      return Quat(Eigen::Quaterniond(rot.eigen()));
    }

    inline double getRoll(const Rot3& r)
    {
      return r.roll();
    }

    inline double getRoll(const Quat& q)
    {
      return Rot3(q).roll();
    }

    inline double getPitch(const Rot3& r)
    {
      return r.pitch();
    }

    inline double getPitch(const Quat& q)
    {
      return Rot3(q).pitch();
    }

    inline double getYaw(const Rot3& r)
    {
      return r.yaw();
    }

    inline double getYaw(const Quat& q)
    {
      return Rot3(q).yaw();
    }

    inline double SO3Error(const Quat& q1, const Quat& q2)
    {
      return Rot3(q1).error(Rot3(q2));
    }

    inline double SO3Error(const Rot3& r1, const Rot3& r2)
    {
      return r1.error(r2);
    }

    inline la::Vec3 cartesianToSpherical(const la::Vec3& v)
    {
      double rho = v.norm();
      return la::Vec3(rho, acos(v.z()/rho), atan2(v.y(), v.x()));
    }

    inline la::Vec3 sphericalToCartesian(const la::Vec3& v)
    {
      return la::Vec3(v(0)*sin(v(1))*cos(v(2)),
                      v(0)*sin(v(1))*sin(v(2)),
                      v(0)*cos(v(1)));
    }

    inline la::Vec3 NEDCartesian(const la::Vec3& v)
    {
      return la::Vec3(v(0), -v(1), -v(2));
    }

  } //\namespace geometry
} //\namespace robotics_toolkit

#endif
