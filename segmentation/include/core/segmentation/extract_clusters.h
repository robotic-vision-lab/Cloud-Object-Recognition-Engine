/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef EXTRACT_CLUSTERS_H 
#define EXTRACT_CLUSTERS_H 

#include <pcl/segmentation/extract_clusters.h>

int euclideanClusterExtraction (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                                std::vector<pcl::PointIndices> &,
                                float, int, int);

int euclideanClusterExtraction (const pcl::search::KdTree<pcl::PointXYZRGB>::Ptr &,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                                std::vector<pcl::PointIndices> &,
                                float, int, int, int*);

#endif  // EXTRACT_CLUSTERS_H 
