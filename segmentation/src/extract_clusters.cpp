/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/segmentation/extract_clusters.h>

int
euclideanClusterExtraction (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &tree,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                            std::vector<pcl::PointIndices> &cluster_indices,
                            float cluster_tolerance,
                            int min_cluster_size, int max_cluster_size)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  ec.setClusterTolerance (cluster_tolerance); 
  ec.setMinClusterSize (min_cluster_size);   
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  
  if (cluster_indices.empty ())
    return (-1);
  
  return (0);
}

int
euclideanClusterExtraction (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &tree,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                            std::vector<pcl::PointIndices> &cluster_indices,
                            float cluster_tolerance,
                            int min_cluster_size, int max_cluster_size,
                            int* offset)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  ec.setClusterTolerance (cluster_tolerance); 
  ec.setMinClusterSize (min_cluster_size);   
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  
  if (cluster_indices.empty ())
    return (-1);
  
  // Find the closest extracted cloud within a cluster 
  double global_min = 1e16;
  std::vector<double> z_min;
  std::vector<int>::const_iterator pit;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    double min = 1e16;
    for (pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      if (cloud_filtered->points[*pit].z < min)
        min = cloud_filtered->points[*pit].z;
      if (cloud_filtered->points[*pit].z < global_min)
        global_min = cloud_filtered->points[*pit].z;
    }
    z_min.push_back (min);
  }
          
  *offset = 0;
  for (std::vector<double>::const_iterator it = z_min.begin (); it != z_min.end (); ++it)
  {
    if (*it == global_min)
      break;
    *offset++;
  }

  return (0);
}
