#include <core_server/server_pointcloud_processor.h>

struct timeval cloudcallback_start, cloudcallback_end;

ServerPointCloudProcessor::ServerPointCloudProcessor():
  ph_("~"),
  input_("/core_server_pointcloud/output"),
  enable_range_(true),
  enable_plane_(true),
  min_distance_(0),  
  max_distance_(1.5),    
  distance_threshold_(0.015)
{
  ph_.param("input", input_, input_);
  ph_.param("enable_range", enable_range_, enable_range_);
  ph_.param("enable_plane", enable_plane_, enable_plane_);
  ph_.param("min_distance", min_distance_, min_distance_);
  ph_.param("max_distance", max_distance_, max_distance_);
  ph_.param("distance_threshold", distance_threshold_, distance_threshold_);
  
  sub_ = nh_.subscribe(input_, 1, &ServerPointCloudProcessor::cloudCallback, this);
}

void
ServerPointCloudProcessor::decompressPointCloud(std::stringstream &compressed_data, 
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* decoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
  decoder->decodePointCloud(compressed_data, cloud);
  delete(decoder);
}

void
ServerPointCloudProcessor::cloudCallback(const core_msgs::PointCloudConstPtr& input)
{
  bool is_filtered = false;

#if 0
  // Measure the elapsed time in milliseconds between callbacks
  long mtime, seconds, useconds;
  gettimeofday(&cloudcallback_end, NULL);
  seconds  = filter_range_end.tv_sec  - filter_range_start.tv_sec;
  useconds = filter_range_end.tv_usec - filter_range_start.tv_usec;
  mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
  std::cout << "cloudCallback elapsed time: " << mtime << " ms" << std::endl;
#endif

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::stringstream compressed_data(input->data);
  decompressPointCloud(compressed_data, cloud);

  if (enable_range_) 
  {
    filterRangeDepth(cloud, cloud_filtered, min_distance_, max_distance_);
    is_filtered = true;
  }

  if (enable_plane_)
  {
    if (filterPlaneModel(cloud_filtered, distance_threshold_) < 0)
      std::cerr << "Could not estimate a planar model for the given data" << std::endl;
    else 
      is_filtered = true;
  }

  // Enqueue point cloud

#if 0
  gettimeofday(&cloudcallback_start, NULL);
#endif
}

int 
main(int argc, char **argv) 
{
  ros::init(argc, argv, "server_pointcloud_processor");
  ServerPointCloudProcessor server_pointcloud_processor;
  ros::spin();
}
