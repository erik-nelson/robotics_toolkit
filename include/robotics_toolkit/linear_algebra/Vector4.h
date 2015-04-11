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

#ifndef __ROBOTICS_TOOLKIT_VECTOR4_H__
#define __ROBOTICS_TOOLKIT_VECTOR4_H__

#include "VectorNBase.h"

namespace robotics_toolkit
{
  namespace linear_algebra
  {
    template<typename T>
    struct Vector4Base : VectorNBase<T, 4>
    {
      Vector4Base() : VectorNBase<T, 4>() { }
      Vector4Base(T val) : VectorNBase<T, 4>(val) { }
      Vector4Base(const Vector4Base& in) : VectorNBase<T, 4>(in.data) { }
      Vector4Base(const boost::array<T, 4>& in) : VectorNBase<T, 4>(in) { }
      Vector4Base(T (&in)[4]) : VectorNBase<T, 4>(in) { }
      Vector4Base(const arma::vec::fixed<4>& in) : VectorNBase<T, 4>(in) { }
      Vector4Base(const Eigen::Matrix<T, 4, 1>& in) : VectorNBase<T, 4>(in) { }
      Vector4Base(const VectorNBase<T, 4>& in) : VectorNBase<T, 4>(in) { }

      Vector4Base(T v1, T v2, T v3, T v4)
      {
        this->data[0] = v1;
        this->data[1] = v2;
        this->data[2] = v3;
        this->data[3] = v4;
      }
    }; //\struct Vector4Base

    inline Vector4Base<float> operator*(const float& lhs, const Vector4Base<float>& rhs)
    {
      return Vector4Base<float>(rhs*lhs);
    }

    inline Vector4Base<double> operator*(const double& lhs, const Vector4Base<double>& rhs)
    {
      return Vector4Base<double>(rhs*lhs);
    }

    typedef Vector4Base<float> Vector4f;
    typedef Vector4Base<float> Vec4f;

    typedef Vector4Base<double> Vector4d;
    typedef Vector4Base<double> Vec4d;

    typedef Vector4Base<double> Vector4;
    typedef Vector4Base<double> Vec4;

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
