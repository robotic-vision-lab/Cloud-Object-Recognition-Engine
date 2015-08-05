/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef PREDICT_SVM_COVARIANCE_CLASS_H
#define PREDICT_SVM_COVARIANCE_CLASS_H

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <core/console/print.h>
#include <core/filters/filter_range.h>
#include <core/filters/filter_model.h>
#include <core/segmentation/extract_clusters.h>
#include <core/features/normals.h>
#include <core/descriptors/covariance.h>
#include "svm.h"

int predictClass (const std::string, const std::string);

#endif  // PREDICT_SVM_COVARIANCE_CLASS_H
