/*
 * Copyright (C) 2015 - Erik Nelson
 * Copyright (C) 2013 - Nathan Michael
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#ifndef __ROBOTICS_TOOLKIT_VECTOR3_H__
#define __ROBOTICS_TOOLKIT_VECTOR3_H__

#include "VectorNBase.h"

namespace robotics_toolkit
{
  namespace linear_algebra
  {

    template<typename T>
    struct Vector3Base : VectorNBase<T, 3>
    {
      Vector3Base() : VectorNBase<T, 3>() { }
      Vector3Base(T val) : VectorNBase<T, 3>(val) { }
      Vector3Base(const Vector3Base& in) : VectorNBase<T, 3>(in.data) { }
      Vector3Base(const boost::array<T, 3>& in) : VectorNBase<T, 3>(in) { }
      Vector3Base(T (&in)[3]) : VectorNBase<T, 3>(in) { }
      Vector3Base(const arma::vec::fixed<3>& in) : VectorNBase<T, 3>(in) { }
      Vector3Base(const Eigen::Matrix<T, 3, 1>& in) : VectorNBase<T, 3>(in) { }
      Vector3Base(const VectorNBase<T, 3>& in) : VectorNBase<T, 3>(in) { }

      Vector3Base(T v1, T v2, T v3)
      {
        this->data[0] = v1;
        this->data[1] = v2;
        this->data[2] = v3;
      }

      T x() const { return this->data[0]; }
      T y() const { return this->data[1]; }
      T z() const { return this->data[2]; }

      inline Vector3Base<T> cross(const Vector3Base<T>& v) const
      {
        return Vector3Base<T>(-(*this)(2)*v(1) + (*this)(1)*v(2),
                              (*this)(2)*v(0) - (*this)(0)*v(2),
                              -(*this)(1)*v(0) + (*this)(0)*v(1));
      }
    }; //\struct Vector3Base

    inline Vector3Base<float> operator*(const float& lhs, const Vector3Base<float>& rhs)
    {
      return Vector3Base<float>(rhs*lhs);
    }

    inline Vector3Base<double> operator*(const double& lhs, const Vector3Base<double>& rhs)
    {
      return Vector3Base<double>(rhs*lhs);
    }

    template<typename T>
    inline VectorNBase<T, 3> cross(const VectorNBase<T, 3>& v1,
                                   const VectorNBase<T, 3>& v2)
    {
      return Vector3Base<T>(v1).cross(v2);
    }

    typedef Vector3Base<float> Vector3f;
    typedef Vector3Base<float> Vec3f;

    typedef Vector3Base<double> Vector3d;
    typedef Vector3Base<double> Vec3d;

    typedef Vector3Base<double> Vector3;
    typedef Vector3Base<double> Vec3;

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
