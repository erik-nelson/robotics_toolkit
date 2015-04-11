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

#ifndef __ROBOTICS_TOOLKIT_VECTORNBASE_H__
#define __ROBOTICS_TOOLKIT_VECTORNBASE_H__

#include <string>
#include <ostream>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <armadillo>
#include <Eigen/Core>
#include <robotics_toolkit/math/MathDefinitions.h>

namespace robotics_toolkit
{
  namespace linear_algebra
  {

    template<typename T, size_t N>
    struct VectorNBase
    {
      typedef typename boost::shared_ptr< VectorNBase<T, N> > Ptr;
      typedef typename boost::shared_ptr< const VectorNBase<T, N> > ConstPtr;

      static const size_t length = N;

      boost::array<T, N> data;

      VectorNBase() { data.fill(0); }

      VectorNBase(T val) { data.fill(val); }

      VectorNBase(const VectorNBase& in) : data(in.data) { }

      VectorNBase(const boost::array<T, N>& in) : data(in) { }

      VectorNBase(T (&in)[N])
      {
        for (size_t i = 0; i < N; i++)
          data[i] = in[i];
      }

      VectorNBase(const arma::vec::fixed<N>& in)
      {
        for (size_t i = 0; i < N; i++)
          data[i] = in(i);
      }

      VectorNBase(const Eigen::Matrix<T, N, 1>& in)
      {
        for (size_t i = 0; i < N; i++)
          data[i] = in(i, 0);
      }

      inline T& operator()(unsigned int i) { return data[i]; }
      inline const T& operator()(unsigned int i) const { return data[i]; }

      inline T& get(unsigned int i) { return data[i]; }
      inline const T& get(unsigned int i) const { return data[i]; }

      inline VectorNBase& operator=(const VectorNBase& rhs)
      {
        if (this == &rhs)
          return *this;
        data = rhs.data;
        return *this;
      }

      inline VectorNBase operator*(T rhs) const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = data[i]*rhs;
        return VectorNBase<T, N>(d);
      }

      inline VectorNBase operator+(const VectorNBase& rhs) const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = data[i] + rhs.data[i];
        return VectorNBase<T, N>(d);
      }

      inline VectorNBase operator-() const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = -data[i];
        return VectorNBase<T, N>(d);
      }

      inline VectorNBase operator-(const VectorNBase& rhs) const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = data[i] - rhs.data[i];
        return VectorNBase<T, N>(d);
      }

      inline VectorNBase operator%(const VectorNBase& rhs) const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = data[i]*rhs.data[i];
        return VectorNBase<T, N>(d);
      }

      inline T operator^(const VectorNBase& rhs) const
      {
        T dot = 0;
        for (size_t i = 0; i < N; i++)
          dot += data[i]*rhs.data[i];
        return dot;
      }

      inline VectorNBase operator/(const VectorNBase& rhs) const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = data[i]/rhs.data[i];
        return VectorNBase<T, N>(d);
      }

      inline VectorNBase operator+=(const VectorNBase& rhs)
      {
        for (size_t i = 0; i < N; i++)
          data[i] += rhs.data[i];
        return *this;
      }

      inline VectorNBase operator-=(const VectorNBase& rhs)
      {
        for (size_t i = 0; i < N; i++)
          data[i] -= rhs.data[i];
        return *this;
      }

      inline VectorNBase operator%=(const VectorNBase& rhs)
      {
        for (size_t i = 0; i < N; i++)
          data[i] *= rhs.data[i];
        return *this;
      }

      inline VectorNBase operator/=(const VectorNBase& rhs)
      {
        for (size_t i = 0; i < N; i++)
          data[i] /= rhs.data[i];
        return *this;
      }

      inline VectorNBase operator*=(const T& rhs)
      {
        for (size_t i = 0; i < N; i++)
          data[i] *= rhs;
        return *this;
      }

      inline VectorNBase operator/=(const T& rhs)
      {
        for (size_t i = 0; i < N; i++)
          data[i] /= rhs;
        return *this;
      }

      inline bool operator==(const VectorNBase& that) const
      {
        return this->equals(that);
      }

      inline bool operator!=(const VectorNBase& that) const
      {
        return !this->equals(that);
      }

      inline bool equals(const VectorNBase& that, const T ptol = 1e-8) const
      {
        return (*this - that).norm() < ptol;
      }

      inline T norm() const
      {
        return math::sqrt((*this)^(*this));
      }

      inline VectorNBase normalize() const
      {
        return (*this)/norm();
      }

      inline VectorNBase abs() const
      {
        T d[N];
        for (size_t i = 0; i < N; i++)
          d[i] = std::abs(data[i]);
        return VectorNBase<T, N>(d);
      }

      inline void ones()
      {
        data.fill(1);
      }

      inline void zeros()
      {
        data.fill(0);
      }

      inline T dot(const VectorNBase& v) const
      {
        return (*this)^v;
      }

      inline VectorNBase scale(T s) const
      {
        return (*this)*s;
      }

      inline void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          std::cout << prefix << std::endl;
        std::cout << (*this) << std::endl;
      }

      inline arma::vec::fixed<N> arma() const
      {
        return arma::vec::fixed<N>(data.data());
      }

      inline Eigen::Matrix<T, N, 1> eigen() const
      {
        return Eigen::Matrix<T, N, 1>(data.data());
      }
    }; //\struct VectorNBase

    template<typename T, size_t N>
    inline VectorNBase<T, N> operator*(const T& lhs, const VectorNBase<T, N>& rhs)
    {
      return rhs*lhs;
    }

    template<typename T, size_t N>
    inline std::ostream& operator<<(std::ostream& out, const VectorNBase<T, N>& m)
    {
      for (size_t i = 0; i < N - 1; i++)
        out << m.data[i] << " ";
      out << m.data[N-1];
      return out;
    }

    template<typename T, size_t N>
    inline arma::vec::fixed<N> arma(const VectorNBase<T, N>& in)
    {
      return in.arma();
    }

    template<typename T, size_t N>
    inline Eigen::Matrix<T, N, 1> eigen(const VectorNBase<T, N>& in)
    {
      return in.eigen();
    }

    template<typename T, size_t N>
    inline T norm(const VectorNBase<T, N>& v)
    {
      return v.norm();
    }

    template<typename T, size_t N>
    inline T dot(const VectorNBase<T, N>& v1, const VectorNBase<T, N>& v2)
    {
      return v1.dot(v2);
    }

  } //\namespace linear_algebra
} //\namespace robotics_toolkit

#endif
