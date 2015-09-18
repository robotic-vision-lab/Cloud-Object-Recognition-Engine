/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/segmentation/extract_clusters.h>

int
euclideanClusterExtraction (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &tree,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                            std::vector<pcl::PointIndices> &cluster_indices,
                            float cluster_tolerance,
                            int min_cluster_size, 
                            int max_cluster_size)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  ec.setClusterTolerance (cluster_tolerance); 
  ec.setMinClusterSize (min_cluster_size);   
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  if (cluster_indices.empty ())
    return (-1);
  
  return (0);
}
