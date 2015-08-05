/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <ctype.h>
#include <stdio.h>
#include <core/console/parse.h>
#include <core/console/print.h>

bool
core::console::find_switch (int argc, char** argv, const char* argument_name)
{
  return (find_argument (argc, argv, argument_name) != -1);
}

int
core::console::find_argument (int argc, char** argv, const char* argument_name)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if (strcmp (argv[i], argument_name) == 0)
    {
      return (i);
    }
  }

  return (-1);
}

int
core::console::parse_argument (int argc, char** argv, const char* str, std::string &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = argv[index];

  return index - 1;
}

int
core::console::parse_argument (int argc, char** argv, const char* str, bool &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = atoi (argv[index]) == 1;

  return (index - 1);
}

int
core::console::parse_argument (int argc, char** argv, const char* str, double &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = atof (argv[index]);

  return (index - 1);
}

int
core::console::parse_argument (int argc, char** argv, const char* str, float &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = static_cast<float> (atof (argv[index]));

  return (index - 1);
}

int
core::console::parse_argument (int argc, char** argv, const char* str, int &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = atoi (argv[index]);

  return (index - 1);
}

int
core::console::parse_argument (int argc, char** argv, const char* str, unsigned int &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = atoi (argv[index]);

  return (index - 1);
}

int
core::console::parse_argument (int argc, char** argv, const char* str, char &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc)
    val = argv[index][0];

  return (index - 1);
}

std::vector<int>
core::console::parse_file_extension_argument (int argc, char** argv, const std::string &extension)
{
  std::vector<int> indices;

  for (int i = 1; i < argc; ++i)
  {
    std::string fname = std::string (argv[i]);
    std::string ext = extension;

    // Needs to be at least 4: .ext
    if (fname.size () <= 4)
      continue;

    // For being case insensitive
    std::transform (fname.begin (), fname.end (), fname.begin (), tolower);
    std::transform (ext.begin (), ext.end (), ext.begin (), tolower);

    // Check if found
    std::string::size_type it;
    if ((it = fname.rfind (ext)) != std::string::npos)
    {
      // Additional check: we want to be able to differentiate between .p and .png
      if ((ext.size () - (fname.size () - it)) == 0)
        indices.push_back (i);
    }
  }

  return (indices);
}

int
core::console::parse_2x_arguments (int argc, char** argv, const char* str, float &f, float &s, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2 && debug)
      {
        print_error ("[parse_2x_arguments] Number of values for %s (%zu) different than 2!\n", str, values.size ());
        return (-2);
      }
      f = static_cast<float> (atof (values.at (0).c_str ()));
      s = static_cast<float> (atof (values.at (1).c_str ()));
      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_2x_arguments (int argc, char** argv, const char* str, double &f, double &s, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2 && debug)
      {
        print_error ("[parse_2x_arguments] Number of values for %s (%zu) different than 2!\n", str, values.size ());
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_2x_arguments (int argc, char** argv, const char* str, int &f, int &s, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2 && debug)
      {
        print_error ("[parse_2x_arguments] Number of values for %s (%zu) different than 2!\n", str, values.size ());
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_3x_arguments (int argc, char** argv, const char* str, float &f, float &s, float &t, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3 && debug)
      {
        print_error ("[parse_3x_arguments] Number of values for %s (%zu) different than 3!\n", str, values.size ());
        return (-2);
      }
      f = static_cast<float> (atof (values.at (0).c_str ()));
      s = static_cast<float> (atof (values.at (1).c_str ()));
      t = static_cast<float> (atof (values.at (2).c_str ()));
      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_3x_arguments (int argc, char** argv, const char* str, double &f, double &s, double &t, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3 && debug)
      {
        print_error ("[parse_3x_arguments] Number of values for %s (%zu) different than 3!\n", str, values.size ());
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_3x_arguments (int argc, char** argv, const char* str, int &f, int &s, int &t, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3 && debug)
      {
        print_error ("[parse_3x_arguments] Number of values for %s (%zu) different than 3!\n", str, values.size ());
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      t = atoi (values.at (2).c_str ());
      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_x_arguments (int argc, char** argv, const char* str, std::vector<double>& v)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);

      v.resize (values.size ());
      for (size_t j = 0; j < v.size (); ++j)
        v[j] = atof (values.at (j).c_str ());

      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_x_arguments (int argc, char** argv, const char* str, std::vector<float>& v)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);

      v.resize (values.size ());
      for (size_t j = 0; j < v.size (); ++j)
        v[j] = static_cast<float> (atof (values.at (j).c_str ()));

      return (i - 1);
    }
  }

  return (-1);
}

int
core::console::parse_x_arguments (int argc, char** argv, const char* str, std::vector<int>& v)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);

      v.resize (values.size ());
      for (size_t j = 0; j < v.size (); ++j)
        v[j] = atoi (values.at (j).c_str ());

      return (i - 1);
    }
  }

  return (-1);
}

bool
core::console::parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<int> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      int val = atoi (argv[i]);
      values.push_back (val);
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

bool
core::console::parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<double> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      double val = atof (argv[i]);
      values.push_back (val);
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

bool
core::console::parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<float> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      float val = static_cast<float> (atof (argv[i]));
      values.push_back (val);
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

bool
core::console::parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<std::string> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      values.push_back (std::string (argv[i]));
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

bool
core::console::parse_multiple_2x_arguments (int argc, char** argv, const char* str, std::vector<double> &values_f, std::vector<double> &values_s)
{
  double f, s;

  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2)
      {
        print_error ("[parse_multiple_2x_arguments] Number of values for %s (%zu) different than 2!\n", str, values.size ());
        return (false);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      values_f.push_back (f);
      values_s.push_back (s);
    }
  }
  if (values_f.size () == 0)
    return (false);
  else
    return (true);
}

bool
core::console::parse_multiple_3x_arguments (int argc, char** argv, const char* str,
                                           std::vector<double> &values_f,
                                           std::vector<double> &values_s,
                                           std::vector<double> &values_t)
{
  double f, s, t;

  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // Look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3)
      {
        print_error ("[parse_multiple_3x_arguments] Number of values for %s (%zu) different than 3!\n", str, values.size ());
        return (false);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      values_f.push_back (f);
      values_s.push_back (s);
      values_t.push_back (t);
    }
  }
  if (values_f.size () == 0)
    return (false);
  else
    return (true);
}
