/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/learning/svm/svm_train.h>
#include <core/configuration/configuration.h>

void
printHelp (int, char** argv)
{
  CORE_INFO ("Syntax is: %s core.cfg (configuration file) categories (directory list of covariances)\n", argv[0]);
}

int 
main (int argc, char** argv)
{
  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  } 

  std::string configuration_file = argv[1];
  std::string category_file_list = argv[2];

  COREConfiguration core_cfg;
  std::vector<std::string> categories;
  std::vector<std::string> covariances;
  std::vector<std::vector<svm_node> > data;
  std::vector<int> labels;
  int ret_val, label = 1;

  if (getConfiguration (configuration_file, core_cfg) < 0)
    return (-1);

  if ((ret_val = getCategories (category_file_list, categories)) < 0)
    return (-1);

  for (std::vector<std::string>::iterator it = categories.begin (); it != categories.end (); ++it)
  {
    if ((ret_val = getData (*it, covariances)) < 0)
      continue;
    while (!covariances.empty ())
    {
      std::ifstream fs;
      svm_node node;
      std::vector<svm_node> nodes;
      double value;
      int index = 1;

      std::string covariance = covariances.back ();
      covariances.pop_back ();
      fs.open (covariance.c_str (), std::ios::in);
      if (!fs.is_open () || fs.fail ())
      {
        CORE_ERROR ("Could not open file '%s'! Error : %s\n", covariance.c_str (), strerror (errno));
        fs.close ();
        continue;
      }
      // Fill the feature vector
      while (fs >> value)
      {
        node.index = index;
        node.value = value;
        nodes.push_back (node);
        ++index;
      }
      // Terminate the feature vector
      node.index = -1;
      nodes.push_back (node);
      data.push_back (nodes);     
      labels.push_back (label); 
      fs.close ();
    }
    // Update the label when changing category
    ++label;
  }
  
  svmTrain (core_cfg.classification.svm.gamma, data, labels);

  return (0);
}
