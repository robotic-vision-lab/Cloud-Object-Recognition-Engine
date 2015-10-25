/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef SVM_TRAIN_H
#define SVM_TRAIN_H

#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fstream>
#include <vector>
#include <cerrno>
#include <svm.h>
#include <core/utils/utils.h>
#include <core/console/print.h>

int svmTrain (double gamma, std::vector<std::vector<svm_node> > data, std::vector<int> labels);

#endif  // SVM_TRAIN_H
