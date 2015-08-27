/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/learning/svm/svm_train_covariances.h>

void
printHelp (int, char** argv)
{
  CORE_INFO ("Syntax is: %s gamma (learning parameter) categories (directory list of covariances)\n", argv[0]);
}

int 
main (int argc, char** argv)
{
  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  } 

  double gamma = static_cast<double> (atof (argv[1]));
  std::string category_file_list = argv[2];

  svmTrainCovariances (gamma, category_file_list);
  
  return (0);
}
