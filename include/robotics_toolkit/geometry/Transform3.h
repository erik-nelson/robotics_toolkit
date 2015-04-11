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

#ifndef __ROBOTICS_TOOLKIT_TRANSFORM3BASE_H__
#define __ROBOTICS_TOOLKIT_TRANSFORM3BASE_H__

#include <robotics_toolkit/linear_algebra/Vector3.h>
#include <robotics_toolkit/geometry/Rotation3.h>
#include <robotics_toolkit/geometry/Transform2.h>

namespace robotics_toolkit
{
  namespace geometry
  {

    namespace la = ::robotics_toolkit::linear_algebra;

    template<typename T>
    struct Transform3Base
    {
      typedef boost::shared_ptr<Transform3Base> Ptr;
      typedef boost::shared_ptr<const Transform3Base> ConstPtr;

      la::Vector3Base<T> translation;
      Rotation3Base<T> rotation;

      Transform3Base()
      {
        translation.zeros();
        rotation.eye();
      }

      Transform3Base(const la::Vector3Base<T>& translation_,
                     const Rotation3Base<T>& rotation_) :
        translation(translation_), rotation(rotation_) { }

      Transform3Base(const Transform3Base<T>& in) :
        translation(in.translation), rotation(in.rotation) { }

      Transform3Base(const Transform2Base<T>& in)
      {
        translation(0) = in.translation(0);
        translation(1) = in.translation(1);
        translation(2) = 0;
        rotation.eye();
        for (unsigned int i = 0; i < 2; i++)
          for (unsigned int j = 0; j < 2; j++)
            rotation(i, j) = in.rotation(i, j);
      }

      Transform3Base& operator=(const Transform3Base& rhs)
      {
        if (this == &rhs)
          return *this;
        translation = rhs.translation;
        rotation = rhs.rotation;
        return *this;
      }

      la::Vector3Base<T> operator*(const la::Vector3Base<T>& p) const
      {
        return rotation*p + translation;
      }

      Transform3Base<T> operator+(const Transform3Base<T>& t) const
      {
        return Transform3Base<T>(translation + rotation*t.translation,
                                 rotation*t.rotation);
      }

      bool operator==(const Transform3Base& that) const
      {
        return this->equals(that);
      }

      bool operator!=(const Transform3Base& that) const
      {
        return !this->equals(that);
      }

      bool equals(const Transform3Base& that,
                  const T ptol = 1e-5,
                  const T rtol = 1e-5) const
      {
        return (translation.equals(that.translation, ptol) &&
                rotation.equals(that.rotation, rtol));
      }

      void print(const std::string& prefix = std::string()) const
      {
        if (!prefix.empty())
          std::cout << prefix << std::endl;
        std::cout << (*this) << std::endl;
      }

      static Transform3Base identity()
      {
        return Transform3Base();
      }
    }; //\struct Transform3Base

    template<typename T>
    std::ostream& operator<<(std::ostream& out, const Transform3Base<T>& m)
    {
      out << "translation:" << std::endl << m.translation << std::endl;
      out << "rotation:" << std::endl << m.rotation;
      return out;
    }

    template<typename T>
    Transform3Base<T> pose_update(const Transform3Base<T>& t1,
                                  const Transform3Base<T>& t2)
    {
      return Transform3Base<T>(t1.translation + t1.rotation*t2.translation,
                               t1.rotation*t2.rotation);
    }

    template<typename T>
    Transform3Base<T> pose_inverse(const Transform3Base<T>& t)
    {
      return Transform3Base<T>(-1.0*t.rotation.trans()*t.translation,
                               t.rotation.trans());
    }

    template<typename T>
    Transform3Base<T> pose_delta(const Transform3Base<T>& t1,
                                 const Transform3Base<T>& t2)
    {
      return Transform3Base<T>(t1.rotation.trans()*(t2.translation - t1.translation),
                               t1.rotation.trans()*t2.rotation);
    }

    typedef Transform3Base<float> Transform3f;
    typedef Transform3Base<double> Transform3d;
    typedef Transform3d Transform3;
    typedef Transform3 Tr3;

  } //\namespace geometry
} //\namespace robotics_toolkit

#endif
