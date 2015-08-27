/*
 * Cloud-based Object Recognition Engine (CORE)
 *  
 */ 

#include <core/classification/svm/svm_covariance_predict_class.h>

void
printHelp (int, char** argv)
{
  CORE_INFO ("Syntax is: %s pcd_file model_file\n", argv[0]);
}

int 
main (int argc, char** argv)
{
  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  } 

  std::string pcd_file = argv[1];
  std::string model_file = argv[2];

  svmCovariancePredictClass (pcd_file, model_file);

  return (0);
}
