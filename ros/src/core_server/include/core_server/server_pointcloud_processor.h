#include <pcl/octree/octree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <core_msgs/PointCloud.h>
#include <core/filters/filter_range.h>
#include <core/filters/filter_plane.h>

#ifndef SERVER_POINTCLOUD_PROCESSOR_H
#define SERVER_POINTCLOUD_PROCESSOR_H

class ServerPointCloudProcessor
{
  public:
    ServerPointCloudProcessor();
    void decompressPointCloud(std::stringstream &compressed_data, 
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud); 

  private:
    void cloudCallback(const core_msgs::PointCloudConstPtr& input);

    std::string input_;          
    bool range_filter_;          // Enable/disable point cloud range filtering 
    bool model_filter_;          // Enable/disable point cloud planar model filtering 
    double min_range_;           // Range filter minimum distance
    double max_range_;           // Range filter maximum distance
    double distance_threshold_;  // Planar model inlier distance

    ros::Subscriber sub_;
    ros::NodeHandle ph_, nh_;
};

#endif  // SERVER_POINTCLOUD_PROCESSOR_H
