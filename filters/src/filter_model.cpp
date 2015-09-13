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
    
  // Remove the points that fit a plane from the cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> eifilter;
  eifilter.setInputCloud (cloud);
  eifilter.setIndices (inliers);
  eifilter.setNegative (true);
  eifilter.filter (*cloud);

  return (0);
}
