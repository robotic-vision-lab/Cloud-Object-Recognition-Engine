/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/features/normals.h>

void
computeNormals (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &tree,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, float radius)
{
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setRadiusSearch (radius);
  ne.compute (*cloud_normals);
}

void
computeIntegralImageNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, 
                             float change_factor, float smoothing_size)
{
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  // The following normal estimation methods are available: COVARIANCE_MATRIX,
  // AVERAGE_3D_GRADIENT, AVERAGE_DEPTH_CHANGE.
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(change_factor);
  ne.setNormalSmoothingSize(smoothing_size);
  ne.setInputCloud (cloud);
  ne.compute (*cloud_normals);
}
