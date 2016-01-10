/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef CURVATURES_H
#define CURVATURES_H

#include <pcl/features/principal_curvatures.h>

void estimatePrincipalCurvatures (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                                  const pcl::PointCloud<pcl::Normal>::Ptr &,
                                  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &,
                                  float);

#endif  // CURVATURES_H
