/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef FILTER_MODEL_H 
#define FILTER_MODEL_H 

#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

int filterPlaneModel (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, float);

#endif  // FILTER_MODEL_H 
