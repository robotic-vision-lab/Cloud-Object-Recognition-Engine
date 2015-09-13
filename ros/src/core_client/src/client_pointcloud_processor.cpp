#include <core_client/client_pointcloud_processor.h>

struct timeval cloudcallback_start, cloudcallback_end; 

ClientPointCloudProcessor::ClientPointCloudProcessor():
  ph_("~"),
  input_("/camera/depth_registered/points"),
  output_("/client_pointcloud_processor/output"),
  cull_(false),
  range_filter_(true),
  model_filter_(true),
  min_range_(0),  
  max_range_(1.5),    
  distance_threshold_(0.015),  
  entropy_threshold_(0.5)
{
  ph_.param("input", input_, input_);
  ph_.param("output", output_, output_);
  ph_.param("cull", cull_, cull_);
  ph_.param("range_filter", range_filter_, range_filter_);
  ph_.param("model_filter", model_filter_, model_filter_);
  ph_.param("min_range", min_range_, min_range_);
  ph_.param("max_range", max_range_, max_range_);
  ph_.param("distance_threshold", distance_threshold_, distance_threshold_);
  ph_.param("entropy_threshold", entropy_threshold_, entropy_threshold_);
  
  sub_ = nh_.subscribe(input_, 1, &ClientPointCloudProcessor::cloudCallback, this);
  pub_ = nh_.advertise<core_msgs::PointCloud>(output_, 1);
}

float
ClientPointCloudProcessor::computeEntropy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered)
{
  float resolution = 0.01f;  // Low resolution
  float entropy = 0, tpoints = 0, npoints;
    
  pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(resolution);
  octree.setInputCloud(cloud_filtered);
  octree.addPointsFromInputCloud();

  pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::LeafNodeIterator it;
  const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::LeafNodeIterator it_end = octree.leaf_end();

  // Find the total number of points in the leafs 
  #pragma omp parallel for reduction(+:tpoints)
  for (it = octree.leaf_begin(); it != it_end; ++it)
  {
      pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
      tpoints += container.getSize();
  }

  // Compute the entropy based on the point density over all leafs 
  #pragma omp parallel for reduction(+:entropy)
  for (it = octree.leaf_begin(); it != it_end; ++it)
  {
      pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
      npoints = container.getSize();
      entropy += (npoints / tpoints) * log2(npoints / tpoints);
  }

  return -entropy;
}

void
ClientPointCloudProcessor::compressPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                                              std::stringstream &compressed_data)
{
  // Set the compression profile
  pcl::io::compression_Profiles_e compression_profile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

  // Compress the point cloud
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile, false);
  encoder->encodePointCloud(cloud_filtered, compressed_data);
  delete(encoder);
}

void
ClientPointCloudProcessor::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  bool is_filtered = false;
  std::stringstream compressed_data;

#if 0
  // Measure the elapsed time in milliseconds between callbacks
  long mtime, seconds, useconds;
  gettimeofday(&cloudcallback_end, NULL);
  seconds  = filter_range_end.tv_sec  - filter_range_start.tv_sec;
  useconds = filter_range_end.tv_usec - filter_range_start.tv_usec;
  mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
  std::cout << "cloudCallback elapsed time: " << mtime << " ms" << std::endl;
#endif

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);

  if (range_filter_) 
  {
    filterRangeDepth(cloud, cloud_filtered, min_range_, max_range_);
    is_filtered = true;
  }

  if (model_filter_)
  {
    if (filterPlaneModel(cloud_filtered, distance_threshold_) < 0)
      std::cerr << "Could not estimate a planar model for the given data" << std::endl;
    else 
      is_filtered = true;
  }

  if (cull_) 
  {
    float entropy;
    static float prev_entropy = 0;

    if (is_filtered) 
      entropy = computeEntropy(cloud_filtered);
    else
      entropy = computeEntropy(cloud);
    float delta = abs(entropy - prev_entropy);
    prev_entropy = entropy;
    if (delta < entropy_threshold_) 
      return;
  }

  if (is_filtered)
    compressPointCloud(cloud_filtered, compressed_data);
  else
    compressPointCloud(cloud, compressed_data);

  // Convert to core_msgs/PointCloud data type
  core_msgs::PointCloud output;
  output.header = input->header;
  output.data = compressed_data.str();

  pub_.publish(output);

#if 0
  gettimeofday(&cloudcallback_start, NULL);
#endif
}

int 
main(int argc, char **argv) 
{
  ros::init(argc, argv, "client_pointcloud_processor");
  ClientPointCloudProcessor client_pointcloud_processor;
  ros::spin();
}
