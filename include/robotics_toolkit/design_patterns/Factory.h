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

#ifndef __ROBOTICS_TOOLKIT_FACTORY_H__
#define __ROBOTICS_TOOLKIT_FACTORY_H__

#include <boost/shared_ptr.hpp>
#include <robotics_toolkit/design_patterns/Uncopyable.h>

namespace robotics_toolkit
{
  namespace design_patterns
  {
    class Factory : public Uncopyable
    {
    public:

      // Force singleton
      static Factory& instance()
      {
        static Factory instance;
        return instance;
      }

      template<typename T>
      boost::shared_ptr<T> create() const
      {
        return boost::shared_ptr<T>(new T());
      }

      template<typename T>
      boost::shared_ptr<const T> createConst() const
      {
        return boost::shared_ptr<const T>(new T());
      }

      template<typename T, typename arg1T>
      boost::shared_ptr<T> create1(const arg1T& arg1) const
      {
        return boost::shared_ptr<T>(new T(arg1));
      }

      template<typename T, typename arg1T>
      boost::shared_ptr<const T> createConst1(const arg1T& arg1) const
      {
        return boost::shared_ptr<const T>(new T(arg1));
      }

      template<typename T, typename arg1T, typename arg2T>
      boost::shared_ptr<T> create2(const arg1T& arg1,
                                   const arg2T& arg2) const
      {
        return boost::shared_ptr<T>(new T(arg1, arg2));
      }

      template<typename T, typename arg1T, typename arg2T>
      boost::shared_ptr<const T> createConst2(const arg1T& arg1,
                                              const arg2T& arg2) const
      {
        return boost::shared_ptr<const T>(new T(arg1, arg2));
      }

      template<typename T, typename arg1T, typename arg2T, typename arg3T>
      boost::shared_ptr<T> create3(const arg1T& arg1,
                                   const arg2T& arg2,
                                   const arg3T& arg3) const
      {
        return boost::shared_ptr<T>(new T(arg1, arg2, arg3));
      }

      template<typename T, typename arg1T, typename arg2T, typename arg3T>
      boost::shared_ptr<const T> createConst3(const arg1T& arg1,
                                              const arg2T& arg2,
                                              const arg3T& arg3) const
      {
        return boost::shared_ptr<const T>(new T(arg1, arg2, arg3));
      }

      template<typename T, typename arg1T, typename arg2T, typename arg3T, typename arg4T>
      boost::shared_ptr<T> create4(const arg1T& arg1,
                                   const arg2T& arg2,
                                   const arg3T& arg3,
                                   const arg4T& arg4) const
      {
        return boost::shared_ptr<T>(new T(arg1, arg2, arg3, arg4));
      }

      template<typename T, typename arg1T, typename arg2T, typename arg3T, typename arg4T>
      boost::shared_ptr<const T> createConst4(const arg1T& arg1,
                                              const arg2T& arg2,
                                              const arg3T& arg3,
                                              const arg4T& arg4) const
      {
        return boost::shared_ptr<const T>(new T(arg1, arg2, arg3, arg4));
      }


    private:
      Factory() { };

    };

  } //\namespace design_patterns
} //\namespace robotics_toolkit

#endif
