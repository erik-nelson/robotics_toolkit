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

#ifndef __ROBOTICS_TOOLKIT_TIME_CONVERSIONS_H__
#define __ROBOTICS_TOOLKIT_TIME_CONVSERIONS_H__

#include <ctime>

namespace robotics_toolkit
{
  namespace timing
  {
    namespace time_conversions
    {
      inline double toMSec(const double& t) { return t * static_cast<double>(1e3); }
      inline double toUSec(const double& t) { return t * static_cast<double>(1e6); }
      inline double toNSec(const double& t) { return t * static_cast<double>(1e9); }
      inline double toMin(const double& t) { return t / 60.0; }
      inline double toHour(const double& t) { return t / 3600.0; }
      inline double toDay(const double& t) { return t / 86400.0; }
      inline double toYear(const double& t) { return t / 31536000.0; }
    } //\namespace time_conversions
  } //\namespace timing
} //\namespace robotics_toolkit

#endif
