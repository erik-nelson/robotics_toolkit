/*
 * Copyright (C) 2015 - Erik Nelson
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

#ifndef __ROBOTICS_TOOLIT_MEMENTO_H__
#define __ROBOTICS_TOOLIT_MEMENTO_H__

#include <string>
#include <boost/shared_ptr.hpp>

namespace robotics_toolkit
{
  namespace design_patterns
  {

    template<typename T>
    class Memento
    {
    public:
      Memento(const std::string& id_)
      {
        id = id_;
      }

      ~Memento() {}

      inline void save(const T& input)
      {
        data =
          boost::shared_ptr<T>(new T(input));
      }

      inline T& load()
      {
        return *data;
      }

      inline const T& load() const
      {
        return *data;
      }

      inline void reinstate(T& originator) const
      {
        originator = *data;
      }

    private:
      boost::shared_ptr<T> data;
      std::string id;
    }; //\class Memento

  } //\namespace design_patterns
} //\namespacerobotics_toolkit

#endif
