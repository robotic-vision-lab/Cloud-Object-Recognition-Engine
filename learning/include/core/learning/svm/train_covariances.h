/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef TRAIN_COVARIANCES_H
#define TRAIN_COVARIANCES_H

#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fstream>
#include <vector>
#include <cerrno>
#include <svm.h>
#include <core/console/print.h>
#include <core/utils/utils.h>

int trainCovariances (double, const std::string);

#endif  // TRAIN_COVARIANCES_H
