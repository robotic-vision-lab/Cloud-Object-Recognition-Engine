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
#include <core/console/print.h>
#include <core/features/normals.h>
#include <core/features/sobel.h>
#include <core/features/curvatures.h>
#include <core/configuration/configuration.h>
#include <unsupported/Eigen/MatrixFunctions>

int computeCovariance (const Covariance &, 
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                       std::vector<pcl::PointIndices>::const_iterator,
                       Eigen::MatrixXd &);
void writeCovariance (std::ofstream &, const Eigen::MatrixXd &);

#endif  // COVARIANCE_H
