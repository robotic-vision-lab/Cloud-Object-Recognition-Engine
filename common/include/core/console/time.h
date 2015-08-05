/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef CORE_TERMINAL_TOOLS_TIME_H_
#define CORE_TERMINAL_TOOLS_TIME_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <core/console/print.h>

namespace core 
{
  namespace console
  {
    class TicToc
    {
      public:

        TicToc () : tictic (), toctoc () {}

        void 
        tic ()
        {
          tictic = boost::posix_time::microsec_clock::local_time ();
        };

        inline double 
        toc ()
        {
          toctoc = boost::posix_time::microsec_clock::local_time ();
          return (static_cast<double> ((toctoc - tictic).total_milliseconds ()));
        };
        
        inline void 
        toc_print ()
        {
          double milliseconds = toc ();
          //int minutes = (int) floor ( seconds / 60.0 );
          //seconds -= minutes * 60.0;
          //if (minutes != 0)
          //{
          //  print_value ("%i", minutes);
          //  print_info (" minutes, ");
          //}
          print_value ("%g", milliseconds);
          print_info (" ms\n");
        };
      
      private:
        boost::posix_time::ptime tictic;
        boost::posix_time::ptime toctoc;
    };
  } 
}

#endif  // CORE_TERMINAL_TOOLS_TIME_H_
