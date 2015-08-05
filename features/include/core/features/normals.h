/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef NORMALS_H 
#define NORMALS_H 

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

void computeNormals (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &,
                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                     pcl::PointCloud<pcl::Normal>::Ptr &, float);

void computeIntegralImageNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                                  pcl::PointCloud<pcl::Normal>::Ptr &,
                                  float, float);

#endif  // NORMALS_H 
