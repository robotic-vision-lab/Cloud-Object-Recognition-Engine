/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef CORE_CONSOLE_PARSE_H_
#define CORE_CONSOLE_PARSE_H_

#include <vector>
#include <string>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <core/core_macros.h>

namespace core
{
  namespace console
  {
    /** \brief Finds whether the argument with name "argument_name" is in the argument list "argv".
      * An example for a widely used switch argument is the "-r" flag for unix commands that indicates whether
      * the command should run recursively or not.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] argument_name the string value to search for
      * \return true if argument was found, false otherwise
      * \note find_switch is simply returning find_argument != -1.
      */
    CORE_EXPORTS bool
    find_switch (int argc, char** argv, const char* argument_name);

    /** \brief Finds the position of the argument with name "argument_name" in the argument list "argv"
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] argument_name the string value to search for
      * \return index of found argument or -1 of arguments does not appear in list
     */
    CORE_EXPORTS int
    find_argument (int argc, char** argv, const char* argument_name);

    /** \brief Template version for parsing arguments. Template parameter needs to have input stream operator overloaded!
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] argument_name the name of the argument to search for
      * \param[out] value The value of the argument
      * \return index of found argument or -1 of arguments does not appear in list
     */
    template<typename Type> int
    parse (int argc, char** argv, const char* argument_name, Type& value)
    {
      int index = find_argument (argc, argv, argument_name) + 1;

      if (index > 0 && index < argc)
      {
        std::istringstream stream;
        stream.clear ();
        stream.str (argv[index]);
        stream >> value;
      }

      return (index - 1);
    }

    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as a string.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, std::string &val);

    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as a boolean.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, bool &val);

    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as a double.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, float &val);
    
    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as a double.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, double &val);

    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as an int.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, int &val);

    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as an unsigned int.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, unsigned int &val);

    /** \brief Parse for a specific given command line argument. Returns the value
      * sent as an int.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      */
    CORE_EXPORTS int
    parse_argument (int argc, char** argv, const char* str, char &val);

    /** \brief Parse for specific given command line arguments (2x values comma
      * separated). Returns the values sent as doubles.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[in] debug whether to print debug info or not
      */
    CORE_EXPORTS int
    parse_2x_arguments (int argc, char** argv, const char* str, float &f, float &s, bool debug = true);

    /** \brief Parse for specific given command line arguments (2x values comma
      * separated). Returns the values sent as doubles.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[in] debug whether to print debug info or not
      */
    CORE_EXPORTS int
    parse_2x_arguments (int argc, char** argv, const char* str, double &f, double &s, bool debug = true);

    /** \brief Parse for specific given command line arguments (2x values comma
      * separated). Returns the values sent as ints.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[in] debug whether to print debug info or not
      */
    CORE_EXPORTS int
    parse_2x_arguments (int argc, char** argv, const char* str, int &f, int &s, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated). Returns the values sent as doubles.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[out] t the third output value
      * \param[in] debug whether to print debug info or not
      */
    CORE_EXPORTS int
    parse_3x_arguments (int argc, char** argv, const char* str, float &f, float &s, float &t, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated). Returns the values sent as doubles.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[out] t the third output value
      * \param[in] debug whether to print debug info or not
      */
    CORE_EXPORTS int
    parse_3x_arguments (int argc, char** argv, const char* str, double &f, double &s, double &t, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated). Returns the values sent as ints.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[out] t the third output value
      * \param[in] debug whether to print debug info or not
      */
    CORE_EXPORTS int
    parse_3x_arguments (int argc, char** argv, const char* str, int &f, int &s, int &t, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated). Returns the values sent as doubles.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] v the vector into which the parsed values will be copied
      */
    CORE_EXPORTS int
    parse_x_arguments (int argc, char** argv, const char* str, std::vector<double>& v);

    /** \brief Parse for specific given command line arguments (N values comma
      * separated). Returns the values sent as ints.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] v the vector into which the parsed values will be copied
      */
    CORE_EXPORTS int
    parse_x_arguments (int argc, char** argv, const char* str, std::vector<float>& v);

    /** \brief Parse for specific given command line arguments (N values comma
      * separated). Returns the values sent as ints.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] v the vector into which the parsed values will be copied
      */
    CORE_EXPORTS int
    parse_x_arguments (int argc, char** argv, const char* str, std::vector<int>& v);

    /** \brief Parse for specific given command line arguments (multiple occurances
      * of the same command line parameter). Returns the values sent as a vector.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values the resultant output values
      */
    CORE_EXPORTS bool
    parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<int> &values);

    /** \brief Parse for specific given command line arguments (multiple occurances
      * of the same command line parameter). Returns the values sent as a vector.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values the resultant output values
      */
    CORE_EXPORTS bool
    parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<float> &values);

    /** \brief Parse for specific given command line arguments (multiple occurances
      * of the same command line parameter). Returns the values sent as a vector.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values the resultant output values
      */
    CORE_EXPORTS bool
    parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<double> &values);

    /** \brief Parse for a specific given command line argument (multiple occurences
      * of the same command line parameter). Returns the value sent as a vector.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] values the resultant output values
      */
    CORE_EXPORTS bool
    parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<std::string> &values);

    /** \brief Parse for specific given command line arguments (multiple occurances
      * of 2x argument groups, separated by commas). Returns 2 vectors holding the
      * given values.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values_f the first vector of output values
      * \param[out] values_s the second vector of output values
      */
    CORE_EXPORTS bool
    parse_multiple_2x_arguments (int argc, char** argv, const char* str, 
                                 std::vector<double> &values_f, 
                                 std::vector<double> &values_s);

    /** \brief Parse for specific given command line arguments (multiple occurances
      * of 3x argument groups, separated by commas). Returns 3 vectors holding the
      * given values.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values_f the first vector of output values
      * \param[out] values_s the second vector of output values
      * \param[out] values_t the third vector of output values
      */
    CORE_EXPORTS bool
    parse_multiple_3x_arguments (int argc, char** argv, const char* str, 
                                 std::vector<double> &values_f, 
                                 std::vector<double> &values_s, 
                                 std::vector<double> &values_t);

    /** \brief Parse command line arguments for file names. Returns a vector with
      * file names indices.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] ext the extension to search for
      */
    CORE_EXPORTS std::vector<int>
    parse_file_extension_argument (int argc, char** argv, const std::string &ext);
  }
}

#endif  // CORE_CONSOLE_PARSE_H_
