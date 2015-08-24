/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/descriptors/covariance/covariance.h>

void
printHelp (int, char** argv)
{
  CORE_INFO ("Syntax is: %s categories_in (directory list of pcd files) covariances_out (directory path to output covariance files)\n", argv[0]);
}

int 
main (int argc, char** argv)
{
  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  } 

  std::string category_file = argv[1];
  std::string covariance_dir = argv[2];

  computeCovariances (category_file, covariance_dir);

  return (0);
}
