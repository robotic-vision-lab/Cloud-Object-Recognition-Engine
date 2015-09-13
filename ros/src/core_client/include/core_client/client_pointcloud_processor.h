#include <pcl/octree/octree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <core_msgs/PointCloud.h>
#include <core/filters/filter_range.h>
#include <core/filters/filter_plane.h>

#ifndef CLIENT_POINTCLOUD_PROCESSOR_H
#define CLIENT_POINTCLOUD_PROCESSOR_H

class ClientPointCloudProcessor
{
  public:
    ClientPointCloudProcessor();
    float computeEntropy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered);
    void compressPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, 
                            std::stringstream &compressed_data);

  private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

    std::string input_;          // RGB-D sensor subscription topic
    std::string output_;         // Point cloud processor publication topic
    bool cull_;                  // Enable/disable point cloud culling
    bool range_filter_;          // Enable/disable point cloud range filtering 
    bool model_filter_;          // Enable/disable point cloud planar model filtering 
    double min_range_;           // Range filter minimum distance
    double max_range_;           // Range filter maximum distance
    double distance_threshold_;  // Planar model inlier distance
    double entropy_threshold_;   // Point cloud culling entropy threshold 

    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::NodeHandle ph_, nh_;
};

#endif  // CLIENT_POINTCLOUD_PROCESSOR_H
