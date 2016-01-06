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
    bool enable_range_;          // Filter range outliers
    bool enable_plane_;          // Filter a planar model
    double min_distance_;        // Minimum outlier range
    double max_distance_;        // Maximum outlier range
    double distance_threshold_;  // Planar model inlier distance

    ros::Subscriber sub_;
    ros::NodeHandle ph_, nh_;
};

#endif  // SERVER_POINTCLOUD_PROCESSOR_H
