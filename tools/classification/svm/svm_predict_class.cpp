/*
 * Cloud-based Object Recognition Engine (CORE)
 *  
 */ 

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <core/filters/filter_range.h>
#include <core/filters/filter_plane.h>
#include <core/configuration/configuration.h>
#include <core/segmentation/extract_clusters.h>
#include <core/descriptors/covariance/covariance.h>
#include <core/classification/svm/svm_predict_class.h>

#define red      0xffff0000
#define green    0xff00ff00
#define blue     0xff0000ff
#define yellow   0xffffff00
#define magenta  0xffff00ff
#define cyan     0xff00ffff

// For timing measurements
#if 0
struct timeval start, end;
gettimeofday (&start, NULL);
gettimeofday (&end, NULL);
fprintf (stdout, "\nnormals wall time = %.2f ms\n",
        (end.tv_sec-start.tv_sec)*1000.0
         + (end.tv_usec - start.tv_usec) / 1000.0);
#endif

std::vector<unsigned int> 
buildColors ()
{
  std::vector<unsigned int> colors;

  colors.push_back (red);
  colors.push_back (green);
  colors.push_back (blue);
  colors.push_back (yellow);
  colors.push_back (magenta);
  colors.push_back (cyan);

  return colors;
}

void
displayPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, 
                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_class_cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer1 (new pcl::visualization::PCLVisualizer ("Input Cloud"));

  viewer1->initCameraParameters ();
  viewer1->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud_filtered);
  viewer1->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "cloud");
  viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  
  Eigen::Vector4f centroid;
  compute3DCentroid (*cloud_filtered, centroid);
  double viewport_x = centroid (0);
  double viewport_y = centroid (1);
  double viewport_z = centroid (2);
  viewer1->setCameraPosition(0, -1, 0, viewport_x, viewport_y, viewport_z, 0, 0, 1);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer2 (new pcl::visualization::PCLVisualizer ("Classified Cloud"));

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> classified_rgb (colored_class_cloud);
  viewer2->addPointCloud<pcl::PointXYZRGB> (colored_class_cloud, classified_rgb, "colored_class_cloud");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_class_cloud");
  viewer2->setCameraPosition (0, -1, 0, viewport_x, viewport_y, viewport_z, 0, 0, 1);
 
  while ((!viewer1->wasStopped ()) && (!viewer2->wasStopped ()))
  {
    viewer1->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }  
}

void
printHelp (int, char** argv)
{
  CORE_INFO ("Syntax is: %s ore.cfg (configuration file) model_file (SVM model) pcd_file (point cloud)\n", argv[0]);
}

int 
main (int argc, char** argv)
{
  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  } 

  std::string configuration_file = argv[1];
  std::string model_file = argv[2];
  std::string pcd_file = argv[3];

  COREConfiguration core_cfg;
  std::vector<int> indices;
  Eigen::MatrixXd covariance_matrix;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr 
    tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    colored_class_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> cluster_indices;
  
  if (getConfiguration (configuration_file, core_cfg) < 0)
    return (-1);

  struct svm_model* model = svm_load_model (model_file.c_str ());
  if (model == NULL)
  {
    CORE_ERROR ("Could not load SVM model file '%s'!\n", model_file.c_str ());
    return (-1);
  } 

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file.c_str (), *cloud) < 0)
  {
    CORE_ERROR ("Could not open file '%s'! Error : %s\n", pcd_file.c_str (), strerror (errno)); 
  }
  
  if (core_cfg.filter.enable_range)
  {
    std::cout << "Filtering range outliers ... ";
    filterRangeDepth (cloud, 
                      cloud_filtered, 
                      core_cfg.filter.range.min_distance, 
                      core_cfg.filter.range.max_distance);
    std::cout << "done." << std::endl;
  }

  if (core_cfg.filter.enable_plane)
  {
    std::cout << "Filtering plane model ... ";
    if (filterPlaneModel (cloud_filtered, core_cfg.filter.plane.distance_threshold) < 0)
    {
      CORE_ERROR ("Could not estimate a planar model for the given data\n"); 
      return (-1);
    }
    std::cout << "done." << std::endl;
  }

  pcl::removeNaNFromPointCloud (*cloud_filtered, *cloud_filtered, indices);
  std::cout << "Segmenting cloud ... ";
  if (euclideanClusterExtraction (tree, 
                                  cloud_filtered, cluster_indices, 
                                  core_cfg.segmentation.euclidean_cluster.tolerance, 
                                  core_cfg.segmentation.euclidean_cluster.min_points, 
                                  core_cfg.segmentation.euclidean_cluster.max_points) < 0) 
  {
    CORE_ERROR ("No cluster found\n");
    return (-1);
  }
  std::cout << "done." << std::endl;

  std::vector<unsigned int> colors = buildColors ();
  colored_class_cloud = cloud_filtered->makeShared ();

  std::cout << "Computing covariance and predicted label for each object ... ";
  int cluster_count = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    if (computeCovariance (core_cfg.descriptor.covariance, cloud_filtered, it, covariance_matrix) < 0)
      continue;
    
    double label = svmCovariancePredictClass (model, covariance_matrix);

    std::cout << "\ncluster: " << cluster_count << ", predicted label: " << label << std::endl;
    cluster_count++;

    // Color according to the class
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      colored_class_cloud->points[*pit].rgba = colors[int (label - 1)];  
  }
  std::cout << "done." << std::endl;
   
  displayPointClouds (cloud_filtered, colored_class_cloud);
  svm_free_and_destroy_model (&model);
  
  return (0);
}
