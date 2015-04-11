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

#ifndef __ROBOTICS_TOOLKIT_MATRIX3X3_H__
#define __ROBOTICS_TOOLKIT_MATRIX3X3_H__

#include "MatrixNxNBase.h"

namespace robotics_toolkit
{
  namespace linear_algebra
  {

    template<typename T>
    struct Matrix3x3Base : MatrixNxNBase<T, 3>
    {
      Matrix3x3Base() : MatrixNxNBase<T, 3>() { }
      Matrix3x3Base(T val) : MatrixNxNBase<T, 3>(val) { }
      Matrix3x3Base(const Matrix3x3Base& in) : MatrixNxNBase<T, 3>(in.data) { }
      Matrix3x3Base(const boost::array<T, 9>& in) : MatrixNxNBase<T, 3>(in) { }
      Matrix3x3Base(T (&in)[9]) : MatrixNxNBase<T, 3>(in) { }
      Matrix3x3Base(const arma::mat::fixed<3, 3>& in) : MatrixNxNBase<T, 3>(in) { }
      Matrix3x3Base(const Eigen::Matrix<T, 3, 3>& in) : MatrixNxNBase<T, 3>(in) { }
      Matrix3x3Base(const MatrixNxNBase<T, 3>& in) : MatrixNxNBase<T, 3>(in) { }
      Matrix3x3Base(const MatrixNxMBase<T, 3, 3>& in) : MatrixNxNBase<T, 3>(in) { }

      Matrix3x3Base(T R11, T R12, T R13,
                    T R21, T R22, T R23,
                    T R31, T R32, T R33)
      {
        this->data[0] = R11;
        this->data[1] = R12;
        this->data[2] = R13;
        this->data[3] = R21;
        this->data[4] = R22;
        this->data[5] = R23;
        this->data[6] = R31;
        this->data[7] = R32;
        this->data[8] = R33;
      }

      inline T det() const
      {
        T a = this->data[0];
        T b = this->data[1];
        T c = this->data[2];
        T d = this->data[3];
        T e = this->data[4];
        T f = this->data[5];
        T g = this->data[6];
        T h = this->data[7];
        T i = this->data[8];
        return (-(c*e*g) + b*f*g + c*d*h - a*f*h - b*d*i + a*e*i);
      }

      virtual inline MatrixNxNBase<T, 3> inv() const
      {
        if (math::fabs(det()) < std::numeric_limits<T>::denorm_min())
          throw std::runtime_error("Matrix3x3Base: appears singular");

        T a = this->data[0];
        T b = this->data[1];
        T c = this->data[2];
        T d = this->data[3];
        T e = this->data[4];
        T f = this->data[5];
        T g = this->data[6];
        T h = this->data[7];
        T i = this->data[8];
        T tmp[9] = {(f*h - e*i)/(c*e*g - b*f*g - c*d*h + a*f*h + b*d*i - a*e*i),
          (c*h - b*i)/(-(c*e*g) + b*f*g + c*d*h - a*f*h - b*d*i + a*e*i),
          (c*e - b*f)/(c*e*g - b*f*g - c*d*h + a*f*h + b*d*i - a*e*i),
          (f*g - d*i)/(-(c*e*g) + b*f*g + c*d*h - a*f*h - b*d*i + a*e*i),
          (c*g - a*i)/(c*e*g - b*f*g - c*d*h + a*f*h + b*d*i - a*e*i),
          (c*d - a*f)/(-(c*e*g) + b*f*g + c*d*h - a*f*h - b*d*i + a*e*i),
          (e*g - d*h)/(c*e*g - b*f*g - c*d*h + a*f*h + b*d*i - a*e*i),
          (b*g - a*h)/(-(c*e*g) + b*f*g + c*d*h - a*f*h - b*d*i + a*e*i),
          (b*d - a*e)/(c*e*g - b*f*g - c*d*h + a*f*h + b*d*i - a*e*i)};
        return MatrixNxNBase<T, 3>(tmp);
      }
    }; //\struct Matrix3x3Base

    inline Matrix3x3Base<float> operator*(const float& lhs, const Matrix3x3Base<float>& rhs)
    {
      return Matrix3x3Base<float>(rhs.scale(lhs));
    }

    inline Matrix3x3Base<double> operator*(const double& lhs, const Matrix3x3Base<double>& rhs)
    {
      return Matrix3x3Base<double>(rhs.scale(lhs));
    }

    typedef Matrix3x3Base<float> Matrix3x3f;
    typedef Matrix3x3Base<float> Mat33f;

    typedef Matrix3x3Base<double> Matrix3x3d;
    typedef Matrix3x3Base<double> Mat33d;

    typedef Matrix3x3Base<double> Matrix3x3;
    typedef Matrix3x3Base<double> Mat33;

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
