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

#ifndef __ROBOTICS_TOOLKIT_UNCOPYABLE_H__
#define __ROBOTICS_TOOLKIT_UNCOPYABLE_H__

namespace robotics_toolkit
{
  namespace design_patterns
  {

    class Uncopyable
    {
    protected:
      Uncopyable()
      { }

      ~Uncopyable()
      { }

    private:
      Uncopyable(const Uncopyable&);
      Uncopyable& operator=(const Uncopyable&);
    }; //\class Uncopyable

  } //\namespace design_patterns
} //\namespace robotics_toolkit

#endif
