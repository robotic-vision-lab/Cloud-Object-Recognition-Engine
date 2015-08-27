/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef SVM_TRAIN_COVARIANCES_H
#define SVM_TRAIN_COVARIANCES_H

#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fstream>
#include <vector>
#include <cerrno>
#include <svm.h>
#include <core/console/print.h>
#include <core/utils/utils.h>

int svmTrainCovariances (double, const std::string);

#endif  // SVM_TRAIN_COVARIANCES_H
