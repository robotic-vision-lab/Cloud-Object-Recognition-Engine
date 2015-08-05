/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef FILTER_RANGE_H 
#define FILTER_RANGE_H 

#include <pcl/common/common_headers.h>
#include <pcl/filters/conditional_removal.h>

void filterDepthRange (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, 
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                       const float, const float);

#endif  // FILTER_RANGE_H 
