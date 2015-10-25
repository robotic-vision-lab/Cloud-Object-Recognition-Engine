/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef SVM_PREDICT_CLASS_H
#define SVM_PREDICT_CLASS_H

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <svm.h>
#include <core/console/print.h>
#include <unsupported/Eigen/MatrixFunctions>

double svmCovariancePredictClass (const struct svm_model*, const Eigen::MatrixXd &);

#endif  // SVM_PREDICT_CLASS_H
