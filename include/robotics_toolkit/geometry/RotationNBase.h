/*
 * Copyright (C) 2015 - Erik Nelson
 * Copyright (C) 2014 - Nathan Michael
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

#ifndef __ROBOTICS_TOOLKIT_ROTATIONNBASE_H__
#define __ROBOTICS_TOOLKIT_ROTATIONNBASE_H__

#include <ostream>
#include <Eigen/Core>
#include <robotics_toolkit/linear_algebra/MatrixNxNBase.h>

namespace robotics_toolkit
{
  namespace geometry
  {

    namespace la = ::robotics_toolkit::linear_algebra;

    template<typename T, size_t N>
    struct RotationNBase : la::MatrixNxNBase<T, N>
    {
      RotationNBase() : la::MatrixNxNBase<T, N>()
      {
        this->eye();
      }

      RotationNBase(const RotationNBase& in) : la::MatrixNxNBase<T, N>(in.data) { }
      RotationNBase(const boost::array<T, N*N>& in) : la::MatrixNxNBase<T, N>(in) { }
      RotationNBase(T (&in)[N*N]) : la::MatrixNxNBase<T, N>(in) { }
      RotationNBase(const arma::mat::fixed<N, N>& in) : la::MatrixNxNBase<T, N>(in) { }
      RotationNBase(const Eigen::Matrix<T, N, N>& in) : la::MatrixNxNBase<T, N>(in) { }
      RotationNBase(const la::MatrixNxNBase<T, N>& in) : la::MatrixNxNBase<T, N>(in) { }

      virtual inline la::MatrixNxNBase<T, N> inv() const
      {
        return this->trans();
      }
    }; //\struct RotationNBase

    template<size_t N>
    inline RotationNBase<float, N> operator*(const float& lhs,
                                             const RotationNBase<float, N>& rhs)
    {
      return RotationNBase<float, N>(rhs*lhs);
    }

    template<size_t N>
    inline RotationNBase<double, N> operator*(const double& lhs,
                                              const RotationNBase<double, N>& rhs)
    {
      return RotationNBase<double, N>(rhs*lhs);
    }

    template<typename T, size_t N>
    inline RotationNBase<T, N> inv(const RotationNBase<T, N>& m)
    {
      return m.inv();
    }

  } //\namespace geometry
} //\namespace robotics_toolkit
#endif
