/*
 * Cloud-based Object Recognition Engine (CORE)
 *  
 */ 

#include <core/classification/svm/svm_predict_class.h>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

double 
svmCovariancePredictClass (const struct svm_model* model, const Eigen::MatrixXd &covariance_matrix)
{
  int k = 0;
  struct svm_node* node = Malloc (struct svm_node, (covariance_matrix.rows () + 1) * covariance_matrix.rows () / 2 + 1); 
  for (int i = 0 ; i < covariance_matrix.rows (); ++i)
  {
    for (int j = i; j < covariance_matrix.cols (); ++j)
    {
      // Non-diagonal entries are multiplied by sqrt (2.0)
      if (j != i) 
      {
        node[k].index = k + 1;
        node[k].value = sqrt (2.0) * covariance_matrix (i, j);
        ++k;
      }
      else
      {
        node[k].index = k + 1;
        node[k].value = covariance_matrix (i, j);
        ++k;
      }
    }
  } 
  // Terminate the vector
  node[k].index = -1;
        
  double label = svm_predict (model, node);
  free (node);

  return (label);
}
