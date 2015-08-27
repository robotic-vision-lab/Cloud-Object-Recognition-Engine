/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef SVM_COVARIANCE_PREDICT_CLASS_H
#define SVM_COVARIANCE_PREDICT_CLASS_H

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <svm.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <core/console/print.h>
#include <core/filters/filter_range.h>
#include <core/filters/filter_model.h>
#include <core/segmentation/extract_clusters.h>
#include <core/features/normals.h>
#include <core/descriptors/covariance/covariance.h>

int svmCovariancePredictClass (const std::string, const std::string);

#endif  // SVM_COVARIANCE_PREDICT_CLASS_H
