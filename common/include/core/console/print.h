/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef CORE_TERMINAL_TOOLS_PRINT_H_
#define CORE_TERMINAL_TOOLS_PRINT_H_

#include <stdio.h>
#include <stdarg.h>
#include <core/core_macros.h>

#define CORE_ALWAYS(...)  core::console::print (core::console::L_ALWAYS, __VA_ARGS__)
#define CORE_ERROR(...)   core::console::print (core::console::L_ERROR, __VA_ARGS__)
#define CORE_WARN(...)    core::console::print (core::console::L_WARN, __VA_ARGS__)
#define CORE_INFO(...)    core::console::print (core::console::L_INFO, __VA_ARGS__)
#define CORE_DEBUG(...)   core::console::print (core::console::L_DEBUG, __VA_ARGS__)
#define CORE_VERBOSE(...) core::console::print (core::console::L_VERBOSE, __VA_ARGS__)

namespace core 
{
  namespace console
  {
    enum TT_ATTIBUTES
    {
      TT_RESET     = 0,
      TT_BRIGHT    = 1,
      TT_DIM       = 2,
      TT_UNDERLINE = 3,
      TT_BLINK     = 4,
      TT_REVERSE   = 7,
      TT_HIDDEN    = 8
    };

    enum TT_COLORS
    {
      TT_BLACK,
      TT_RED,
      TT_GREEN,
      TT_YELLOW,
      TT_BLUE,
      TT_MAGENTA,
      TT_CYAN,
      TT_WHITE
    };

    enum VERBOSITY_LEVEL
    {
      L_ALWAYS,
      L_ERROR,
      L_WARN,
      L_INFO,
      L_DEBUG,
      L_VERBOSE
    };

    /* Set the verbosity level */
    CORE_EXPORTS void 
    setVerbosityLevel (VERBOSITY_LEVEL level);

    /* Get the verbosity level. */
    CORE_EXPORTS VERBOSITY_LEVEL 
    getVerbosityLevel ();

    /* Initialize verbosity level. */
    CORE_EXPORTS bool 
    initVerbosityLevel ();

    /* Is verbosity level enabled? */
    CORE_EXPORTS bool 
    isVerbosityLevelEnabled (VERBOSITY_LEVEL severity);

    /** \brief Change the text color (on either stdout or stderr) with an attr:fg:bg
      * \param stream the output stream (stdout, stderr, etc)
      * \param attribute the text attribute
      * \param fg the foreground color
      * \param bg the background color
      */
    CORE_EXPORTS void 
    change_text_color (FILE *stream, int attribute, int fg, int bg);
    
    /** \brief Change the text color (on either stdout or stderr) with an attr:fg
      * \param stream the output stream (stdout, stderr, etc)
      * \param attribute the text attribute
      * \param fg the foreground color
      */
    CORE_EXPORTS void 
    change_text_color (FILE *stream, int attribute, int fg);

    /** \brief Reset the text color (on either stdout or stderr) to its original state
      * \param stream the output stream (stdout, stderr, etc)
      */
    CORE_EXPORTS void 
    reset_text_color (FILE *stream);

    /** \brief Print a message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param attr the text attribute
      * \param fg the foreground color
      * \param format the message
      */
    CORE_EXPORTS void 
    print_color (FILE *stream, int attr, int fg, const char *format, ...);

    /** \brief Print an info message on stream with colors
      * \param format the message
      */
    CORE_EXPORTS void 
    print_info (const char *format, ...);

    /** \brief Print an info message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print_info (FILE *stream, const char *format, ...);

    /** \brief Print a highlighted info message on stream with colors
      * \param format the message
      */
    CORE_EXPORTS void 
    print_highlight (const char *format, ...);

    /** \brief Print a highlighted info message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print_highlight (FILE *stream, const char *format, ...);

    /** \brief Print an error message on stream with colors
      * \param format the message
      */
    CORE_EXPORTS void 
    print_error (const char *format, ...);

    /** \brief Print an error message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print_error (FILE *stream, const char *format, ...);

    /** \brief Print a warning message on stream with colors
      * \param format the message
      */
    CORE_EXPORTS void 
    print_warn (const char *format, ...);

    /** \brief Print a warning message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print_warn (FILE *stream, const char *format, ...);

    /** \brief Print a debug message on stream with colors
      * \param format the message
      */
    CORE_EXPORTS void 
    print_debug (const char *format, ...);

    /** \brief Print a debug message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print_debug (FILE *stream, const char *format, ...);

    /** \brief Print a value message on stream with colors
      * \param format the message
      */
    CORE_EXPORTS void 
    print_value (const char *format, ...);

    /** \brief Print a value message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print_value (FILE *stream, const char *format, ...);

    /** \brief Print a message on stream
      * \param level the verbosity level
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    CORE_EXPORTS void 
    print (VERBOSITY_LEVEL level, FILE *stream, const char *format, ...);

    /** \brief Print a message
      * \param level the verbosity level
      * \param format the message
      */
    CORE_EXPORTS void 
    print (VERBOSITY_LEVEL level, const char *format, ...);
  }
} 

#endif  // CORE_TERMINAL_TOOLS_PRINT_H_
