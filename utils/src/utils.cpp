/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/utils/utils.h>

void
replaceExt (std::string &s, const std::string &ext) 
{
  std::string::size_type i = s.rfind ('.', s.length ());

  if (i != std::string::npos) 
  {
    s.replace (i + 1, ext.length (), ext);
  }
}

int 
getCategories (const std::string &file_name, std::vector<std::string> &categories)
{
  std::string line;
  std::ifstream fs;

  fs.open (file_name.c_str (), std::ios::in);
  if (!fs.is_open () || fs.fail ())
  {
      CORE_ERROR ("Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror (errno));
      fs.close ();
      return (-1);
  }

  while (!fs.eof ())
  {
    getline (fs, line);
    if (line == "")
      continue;
    categories.push_back (line);
  } 

  fs.close ();

  return (0);
}

int 
getData (const std::string &dir_name, std::vector<std::string> &data)
{
  DIR* dp;
  struct dirent* dirp;

  if ((dp = opendir (dir_name.c_str ())) == NULL) 
  {
    CORE_ERROR ("Could not open directory '%s'! Error : %s\n", dir_name.c_str (), strerror (errno));
    return (-1);
  }

  while ((dirp = readdir (dp)) != NULL) 
  { 
    if ((strcmp (dirp->d_name, ".") == 0) || (strcmp (dirp->d_name, "..") == 0))
      continue;
    data.push_back (dir_name + "/" + dirp->d_name);
  }

  closedir (dp);

  return (0);
}
