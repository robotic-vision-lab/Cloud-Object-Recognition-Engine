/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/features/curvatures.h>

void
estimatePrincipalCurvatures (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &tree,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                             pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr 
                             &principal_curvatures, float radius)
{
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> pce;
  pce.setInputCloud (cloud);
  pce.setInputNormals (cloud_normals);
  pce.setSearchMethod (tree);
  pce.setRadiusSearch (radius);
  pce.compute (*principal_curvatures);
}
