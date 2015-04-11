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

#ifndef __ROBOTICS_TOOLKIT_TIMING_H__
#define __ROBOTICS_TOOLKIT_TIMING_H__

#include <ctime>
#include <stdio.h>
#include <string>

namespace robotics_toolkit
{
  namespace timing
  {

    class Timer
    {
    public:
      explicit Timer()
      {
        clock_gettime(CLOCK_REALTIME, &beg);
      }

      virtual ~Timer()
      {}

      virtual double tic()
      {
        clock_gettime(CLOCK_REALTIME, &beg);
        return beg.tv_sec + beg.tv_nsec / static_cast<double>(1e9);
      }

      virtual double ptic(const std::string& prefix = std::string())
      {
        clock_gettime(CLOCK_REALTIME, &beg);
        double t = beg.tv_sec + beg.tv_nsec / static_cast<double>(1e9);

        if (!prefix.empty())
        {
          fputs(prefix.c_str(), stdout);
          printf(": ");
        }

        printf("Begin: %lf (s)\n", t);
        return t;
      }

      virtual double toc()
      {
        clock_gettime(CLOCK_REALTIME, &end);
        return end.tv_sec - beg.tv_sec + (end.tv_nsec - beg.tv_nsec) / static_cast<double>(1e9);
      }

      virtual double ptoc(const std::string& prefix = std::string())
      {
        clock_gettime(CLOCK_REALTIME, &end);
        double t = end.tv_sec - beg.tv_sec + (end.tv_nsec - beg.tv_nsec) / static_cast<double>(1e9);

        if (!prefix.empty())
        {
          fputs(prefix.c_str(), stdout);
          printf(": ");
        }

        printf("Elapsed: %lf (s)\n", t);
        return t;
      }

    protected:
      timespec beg, end;

    }; //\class Timer

  } //\namespace timing
} //\namespace robotics_toolkit

#endif
