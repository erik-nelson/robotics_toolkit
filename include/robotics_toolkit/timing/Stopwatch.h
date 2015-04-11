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

#ifndef __ROBOTICS_TOOLKIT_STOPWATCH_H__
#define __ROBOTICS_TOOLKIT_STOPWATCH_H__

#include <robotics_toolkit/timing/Timer.h>

namespace robotics_toolkit
{
  namespace timing
  {

    class Stopwatch : public Timer
    {
    public:
      explicit Stopwatch()
      : stopwatch_mode(OFF), elapsed(0.0)
      { }

      virtual ~Stopwatch()
      { }

      virtual double tic()
      {
        if (stopwatch_mode != OFF)
          printf("Stopwatch::tic() : Potentially causing undefined behavior with "
                 "call to Stopwatch::tic() while stopwatch is enabled. "
                 "Call Stopwatch::stopwatch_off() first.\n");

        return Timer::tic();
      }

      virtual double ptic(const std::string& prefix = std::string())
      {
        if (stopwatch_mode != OFF)
          printf("Stopwatch::ptic() : Potentially causing undefined behavior with "
                 "call to Stopwatch::ptic() while stopwatch is enabled. "
                 "Call Stopwatch::stopwatch_off() first.\n");

        return Timer::ptic(prefix);
      }

      virtual double toc()
      {
        if (stopwatch_mode != OFF)
          printf("Stopwatch::toc() : Potentially causing undefined behavior with "
                 "call to Stopwatch::toc() while stopwatch is enabled. "
                 "Call Stopwatch::stopwatch_off() first.\n");

        return Timer::toc();
      }

      virtual double ptoc(const std::string& prefix = std::string())
      {
        if (stopwatch_mode != OFF)
          printf("Stopwatch::toc() : Potentially causing undefined behavior with "
                 "call to Stopwatch::toc() while stopwatch is enabled. "
                 "Call Stopwatch::stopwatch_off() first.\n");

        return Timer::ptoc(prefix);
      }

      // Use the start/stop functionality as a stopwatch
      // For example, to time snippets of code inside of a for loop

      // Start the stopwatch. Time will not start ticking until Timer::start() is called.
      double stopwatchOn()
      {
        stopwatch_mode = ON;
        elapsed = 0.0;

        return elapsed;
      }

      // While reset does the same thing as on(),
      // emulate a real stopwatch by providing both
      // functionalities to the user
      double stopwatchReset()
      {
        return stopwatchOn();
      }

      double start()
      {
        if (stopwatch_mode == ON || stopwatch_mode == STOP)
        {
          stopwatch_mode = START;
          clock_gettime(CLOCK_REALTIME, &beg);
          return beg.tv_sec + beg.tv_nsec / static_cast<double>(1e9);
        }

        if (stopwatch_mode == START)
        {
          printf("Stopwatch::start() : Stopwatch has already been started.\n");
          return beg.tv_sec + beg.tv_nsec / static_cast<double>(1e9);
        }

        if (stopwatch_mode == OFF)
        {
          printf("Stopwatch::start() : Stopwatch is off. Call Stopwatch::stopwatchOn() first.\n");
          return 0.0;
        }
      }

      double stop()
      {
        // Calling stop twice in a row
        // will not add any time to the stop watch
        if (stopwatch_mode == START)
        {
          stopwatch_mode = STOP;
          clock_gettime(CLOCK_REALTIME, &end);
          elapsed += end.tv_sec - beg.tv_sec + (end.tv_nsec - beg.tv_nsec) / static_cast<double>(1e9);
          return elapsed;
        }

        if (stopwatch_mode == STOP)
        {
          printf("Stopwatch::stop() : Stopwatch has already been stopped.\n");
          return elapsed;
        }

        if (stopwatch_mode == OFF)
        {
          printf("Stopwatch::stop() : Stopwatch is off. Call Stopwatch::stopwatchOn() first.\n");
          return 0.0;
        }

        if (stopwatch_mode == ON)
        {
          printf("Stopwatch::stop() : Stopwatch is not started. Call Stopwatch::start() first.\n");
          return 0.0;
        }
      }

      double stopwatchOff()
      {
        stopwatch_mode = OFF;
      }

      double stopwatchPrint(const std::string& prefix = std::string())
      {
        if (!prefix.empty())
        {
          fputs(prefix.c_str(), stdout);
          printf(": ");
        }

        printf("Elapsed: %lf (s)\n", elapsed);
        return elapsed;
      }

      double stopwatchElapsed()
      {
        return elapsed;
      }

    private:
      enum { START, STOP, OFF, ON } stopwatch_mode;

      double elapsed;
    }; //\class Stopwatch

  } //\namespace timing
} //\namespace robotics_toolkit

#endif
