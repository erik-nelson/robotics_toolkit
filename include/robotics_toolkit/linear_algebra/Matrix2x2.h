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

#ifndef __ROBOTICS_TOOLKIT_MATRIX2X2_H__
#define __ROBOTICS_TOOLKIT_MATRIX2X2_H__

#include "MatrixNxNBase.h"
#include "Vector2.h"

namespace robotics_toolkit
{
  namespace linear_algebra
  {

    template<typename T>
    struct Matrix2x2Base : MatrixNxNBase<T, 2>
    {
      Matrix2x2Base() : MatrixNxNBase<T, 2>() { }
      Matrix2x2Base(T val) : MatrixNxNBase<T, 2>(val) { }
      Matrix2x2Base(const Matrix2x2Base& in) : MatrixNxNBase<T, 2>(in.data) { }
      Matrix2x2Base(const boost::array<T, 4>& in) : MatrixNxNBase<T, 2>(in) { }
      Matrix2x2Base(T (&in)[2*2]) : MatrixNxNBase<T, 2>(in) { }
      Matrix2x2Base(const arma::mat::fixed<2, 2>& in) : MatrixNxNBase<T, 2>(in) { }
      Matrix2x2Base(const Eigen::Matrix<T, 2, 2>& in) : MatrixNxNBase<T, 2>(in) { }
      Matrix2x2Base(const MatrixNxNBase<T, 2>& in) : MatrixNxNBase<T, 2>(in) { }
      Matrix2x2Base(const MatrixNxMBase<T, 2, 2>& in) : MatrixNxNBase<T, 2>(in) { }

      Matrix2x2Base(T R11, T R12, T R21, T R22)
      {
        this->data[0] = R11;
        this->data[1] = R12;
        this->data[2] = R21;
        this->data[3] = R22;
      }

      virtual inline T det() const
      {
        T a = this->data[0];
        T b = this->data[1];
        T c = this->data[2];
        T d = this->data[3];
        return (-(b*c) + a*d);
      }

      virtual inline MatrixNxNBase<T, 2> inv() const
      {
        Vector2Base<T> e(singularValues());

        T emax = e(0);
        T emin = e(1);

        if (emin < std::numeric_limits<T>::denorm_min())
          throw std::runtime_error("Matrix2x2Base: appears singular");

        if (emax/emin > std::numeric_limits<T>::epsilon())
        {
          T a = this->data[0];
          T b = this->data[1];
          T c = this->data[2];
          T d = this->data[3];

          T tmp[4] = {d/(-b*c + a*d), b/(b*c - a*d), c/(b*c - a*d), a/(-b*c + a*d)};
          return MatrixNxNBase<T, 2>(tmp);
        }
        else
          throw std::runtime_error("Matrix2x2Base: appears singular");
      }

      virtual inline Vector2Base<T> singularValues() const
      {
        T a = this->data[0];
        T b = this->data[1];
        T c = this->data[2];
        T d = this->data[3];

        T tmp1 = a*a + b*b + c*c + d*d;
        T tmp2 = math::sqrt((math::pow(b + c, static_cast<T>(2)) +
                             math::pow(a - d, static_cast<T>(2)))*
                            (math::pow(b - c, static_cast<T>(2)) +
                             math::pow(a + d, static_cast<T>(2))));

        T e1 = math::sqrt(tmp1 - tmp2)*M_SQRT1_2;
        T e2 = math::sqrt(tmp1 + tmp2)*M_SQRT1_2;

        return Vector2Base<T>(e1 > e2 ? e1 : e2, e1 < e2 ? e1 : e2);
      }
    }; //\struct Matrix2x2Base

    inline Matrix2x2Base<float> operator*(const float& lhs, const Matrix2x2Base<float>& rhs)
    {
      return Matrix2x2Base<float>(rhs*lhs);
    }

    inline Matrix2x2Base<double> operator*(const double& lhs, const Matrix2x2Base<double>& rhs)
    {
      return Matrix2x2Base<double>(rhs*lhs);
    }

    typedef Matrix2x2Base<float> Matrix2x2f;
    typedef Matrix2x2Base<float> Mat22f;

    typedef Matrix2x2Base<double> Matrix2x2d;
    typedef Matrix2x2Base<double> Mat22d;

    typedef Matrix2x2Base<double> Matrix2x2;
    typedef Matrix2x2Base<double> Mat22;

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
