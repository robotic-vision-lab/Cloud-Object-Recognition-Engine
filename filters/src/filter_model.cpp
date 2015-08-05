/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/filters/filter_model.h>

int
filterPlaneModel (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float distance)
{
  Eigen::VectorXf planecoeffs, coeffs;
  pcl::PointIndices::Ptr
    inliers (new pcl::PointIndices);
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
  ransac.setDistanceThreshold (distance);
  ransac.computeModel ();
  ransac.getInliers (inliers->indices);
  ransac.getModelCoefficients (planecoeffs);

  model_p->optimizeModelCoefficients (inliers->indices, planecoeffs, coeffs);
  model_p->selectWithinDistance (coeffs, distance, inliers->indices);

  if (inliers->indices.empty ())
    return (-1);

  #pragma omp parallel for
  for (size_t i = 0; i < inliers->indices.size (); ++i)
  {
    cloud->points[inliers->indices[i]].x = std::numeric_limits<double>::quiet_NaN ();
    cloud->points[inliers->indices[i]].y = std::numeric_limits<double>::quiet_NaN ();
    cloud->points[inliers->indices[i]].z = std::numeric_limits<double>::quiet_NaN ();
  }

  return (0);
}
