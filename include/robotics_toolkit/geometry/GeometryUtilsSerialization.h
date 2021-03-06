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

#ifndef __ROBOTICS_TOOLKIT_SERIALIZATION_H__
#define __ROBOTICS_TOOLKIT_SERIALIZATION_H__

#include <robotics_toolkit/geometry/GeometryUtils.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace boost
{
  namespace serialization
  {

    template<class Archive>
    void serialize(Archive & ar, geometry_utils::Vector2& v,
                   const unsigned int version)
    {
      ar & v(0);
      ar & v(1);
    }

    template<class Archive>
    void serialize(Archive & ar, geometry_utils::Vector3& v,
                   const unsigned int version)
    {
      ar & v(0);
      ar & v(1);
      ar & v(2);
    }

    template<class Archive>
    void serialize(Archive & ar, geometry_utils::Vector4& v,
                   const unsigned int version)
    {
      ar & v(0);
      ar & v(1);
      ar & v(2);
      ar & v(3);
    }

    template<class Archive>
    void serialize(Archive & ar, geometry_utils::Matrix3x3& m,
                   const unsigned int version)
    {
      for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
          ar & m(i, j);
    }

    template<class Archive>
    void serialize(Archive & ar, geometry_utils::Transform& t,
                   const unsigned int version)
    {
      ar & t.translation;
      ar & t.rotation;
    }

  } //\namespace serialization
} //\namespace boost

#endif
