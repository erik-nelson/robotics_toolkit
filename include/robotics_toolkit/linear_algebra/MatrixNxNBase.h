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

#ifndef __ROBOTICS_TOOLKIT__MATRIXNXN_H__
#define __ROBOTICS_TOOLKIT__MATRIXNXN_H__

#include "MatrixNxMBase.h"

namespace robotics_toolkit
{
  namespace linear_algebra
  {

    template<typename T, size_t N>
    struct MatrixNxNBase : MatrixNxMBase<T, N, N>
    {
      MatrixNxNBase() : MatrixNxMBase<T, N, N>() { }
      MatrixNxNBase(T val) : MatrixNxMBase<T, N, N>(val) { }
      MatrixNxNBase(const MatrixNxNBase& in) : MatrixNxMBase<T, N, N>(in.data) { }
      MatrixNxNBase(const boost::array<T, N*N>& in) : MatrixNxMBase<T, N, N>(in) { }
      MatrixNxNBase(T (&in)[N*N]) : MatrixNxMBase<T, N, N>(in) { }
      MatrixNxNBase(const arma::mat::fixed<N, N>& in) : MatrixNxMBase<T, N, N>(in) { }
      MatrixNxNBase(const Eigen::Matrix<T, N, N>& in) : MatrixNxMBase<T, N, N>(in) { }
      MatrixNxNBase(const MatrixNxMBase<T, N, N>& in) : MatrixNxMBase<T, N, N>(in) { }

      inline void eye()
      {
        this->data.fill(0);
        for (size_t i = 0; i < this->nrows; i++)
          this->data[this->nrows*i + i] = 1;
      }

      virtual inline T det() const
      {
        std::cerr << "MatrixNxMBase::det not implemented" << std::endl;
        return T();
      }

      virtual inline MatrixNxNBase<T, N> inv() const
      {
        std::cerr << "MatrixNxMBase::inv not implemented" << std::endl;
        return MatrixNxNBase<T, N>();
      }

      static inline MatrixNxNBase<T, N> diagmat(const VectorNBase<T, N>& in)
      {
        T d[N*N] = { 0 };
        for (size_t i = 0; i < N; i++)
          d[N*i + i] = in(i);
        return MatrixNxNBase<T, N>(d);
      }
    }; //\struct MatrixNxNBase

    template<size_t N>
    inline MatrixNxNBase<float, N> operator*(const float& lhs,
                                             const MatrixNxNBase<float, N>& rhs)
    {
      return MatrixNxNBase<float, N>(rhs*lhs);
    }

    template<size_t N>
    inline MatrixNxNBase<double, N> operator*(const double& lhs,
                                              const MatrixNxNBase<double, N>& rhs)
    {
      return MatrixNxNBase<double, N>(rhs*lhs);
    }

    template<typename T, size_t N>
    inline MatrixNxNBase<T, N> inv(const MatrixNxNBase<T, N>& m)
    {
      return m.inv();
    }

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
