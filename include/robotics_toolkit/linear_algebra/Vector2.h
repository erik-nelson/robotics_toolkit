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

#ifndef __ROBOTICS_TOOLKIT_VECTOR2_H__
#define __ROBOTICS_TOOLKIT_VECTOR2_H__

#include "VectorNBase.h"

namespace robotics_toolkit
{
  namespace linear_algebra
  {

    template<typename T>
    struct Vector2Base : VectorNBase<T, 2>
    {
      Vector2Base() : VectorNBase<T, 2>() { }
      Vector2Base(T val) : VectorNBase<T, 2>(val) { }
      Vector2Base(const Vector2Base& in) : VectorNBase<T, 2>(in.data) { }
      Vector2Base(const boost::array<T, 2>& in) : VectorNBase<T, 2>(in) { }
      Vector2Base(T (&in)[2]) : VectorNBase<T, 2>(in) { }
      Vector2Base(const arma::vec::fixed<2>& in) : VectorNBase<T, 2>(in) { }
      Vector2Base(const Eigen::Matrix<T, 2, 1>& in) : VectorNBase<T, 2>(in) { }
      Vector2Base(const VectorNBase<T, 2>& in) : VectorNBase<T, 2>(in) { }

      Vector2Base(T v1, T v2)
      {
        this->data[0] = v1;
        this->data[1] = v2;
      }

      T x() const { return this->data[0]; }
      T y() const { return this->data[1]; }
    }; //\struct Vector2Base

    inline Vector2Base<float> operator*(const float& lhs, const Vector2Base<float>& rhs)
    {
      return Vector2Base<float>(rhs*lhs);
    }

    inline Vector2Base<double> operator*(const double& lhs, const Vector2Base<double>& rhs)
    {
      return Vector2Base<double>(rhs*lhs);
    }

    typedef Vector2Base<float> Vector2f;
    typedef Vector2Base<float> Vec2f;

    typedef Vector2Base<double> Vector2d;
    typedef Vector2Base<double> Vec2d;

    typedef Vector2Base<double> Vector2;
    typedef Vector2Base<double> Vec2;

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
