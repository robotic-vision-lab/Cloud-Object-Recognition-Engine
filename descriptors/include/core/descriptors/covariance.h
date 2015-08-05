/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef COVARIANCE_H
#define COVARIANCE_H

#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <core/console/print.h>
#include <core/filters/filter_range.h>
#include <core/filters/filter_model.h>
#include <core/segmentation/extract_clusters.h>
#include <core/features/normals.h>

int computeCovariance (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                       const pcl::PointCloud<pcl::Normal>::Ptr &,
                       const std::vector<pcl::PointIndices> &,
                       const std::vector<pcl::PointIndices>::const_iterator,
                       Eigen::MatrixXd &);

int computeCovariances (const std::string, const std::string);

#endif  // COVARIANCE_H
