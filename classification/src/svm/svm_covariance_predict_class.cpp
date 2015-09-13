/*
 * Cloud-based Object Recognition Engine (CORE)
 *  
 */ 

#include <core/classification/svm/svm_covariance_predict_class.h>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

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

#if 0
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
  viewer1->camera_.view[0] = 0;
  viewer1->camera_.view[1] = 0;
  viewer1->camera_.view[2] = 1;
  viewer1->camera_.pos[0] = 0;
  viewer1->camera_.pos[1] = -1;
  viewer1->camera_.pos[2] = 0;
  Eigen::Vector4f centroid;
  compute3DCentroid (*cloud_filtered, centroid);
  viewer1->camera_.focal[0] = centroid (0);
  viewer1->camera_.focal[1] = centroid (1);
  viewer1->camera_.focal[2] = centroid (2);
  viewer1->updateCamera();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer2 (new pcl::visualization::PCLVisualizer ("Classified Cloud"));

  viewer2->initCameraParameters ();
  viewer2->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> classified_rgb (colored_class_cloud);
  viewer2->addPointCloud<pcl::PointXYZRGB> (colored_class_cloud, classified_rgb, "colored_class_cloud");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_class_cloud");
  viewer2->camera_.view[0] = 0;
  viewer2->camera_.view[1] = 0;
  viewer2->camera_.view[2] = 1;
  viewer2->camera_.pos[0] = 0;
  viewer2->camera_.pos[1] = -1;
  viewer2->camera_.pos[2] = 0;
  viewer2->camera_.focal[0] = centroid (0);
  viewer2->camera_.focal[1] = centroid (1);
  viewer2->camera_.focal[2] = centroid (2);
  viewer2->updateCamera ();
  
  while ((!viewer1->wasStopped ()) && (!viewer2->wasStopped ()))
  {
    viewer1->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }  
}
#endif

int
svmCovariancePredictClass (const std::string pcd_file, const std::string model_file)
{
  struct svm_node* node; 
  struct svm_model* model;
  Eigen::MatrixXd covariance_matrix;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr 
    tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr 
    normals_cloud (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    colored_class_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> cluster_indices;

  model = svm_load_model(model_file.c_str ());
  if (model == NULL)
  {
    CORE_ERROR ("Could not load SVM model file '%s'!\n", model_file.c_str ());
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file.c_str (), *cloud) < 0)
  {
    CORE_ERROR ("Could not open file '%s'! Error : %s\n", pcd_file.c_str (), strerror (errno)); 
  }
  
  std::cout << "Filtering range outliers ... ";
  filterRangeDepth (cloud, cloud_filtered, 0.0, 1.5);
  std::cout << "done." << std::endl;

  std::cout << "Filtering plane model ... ";
  if (filterPlaneModel (cloud_filtered, 0.015) < 0)
  {
    CORE_ERROR ("Could not estimate a planar model for the given data\n"); 
    return (-1);
  }
  std::cout << "done." << std::endl;

  std::cout << "Computing normals ... ";
  computeIntegralImageNormals(cloud_filtered, normals_cloud, 0.02, 10.0);
  std::cout << "done." << std::endl;

  std::cout << "Segmenting cloud ... ";
  if (euclideanClusterExtraction (tree,
                                  cloud_filtered, cluster_indices,
                                  0.05, 500, 75000) < 0)
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
    int k = 0;
    if (computeCovariance (cloud_filtered, normals_cloud, cluster_indices, it, covariance_matrix) < 0)
      continue;

    node = Malloc (struct svm_node, (covariance_matrix.rows () + 1) * covariance_matrix.rows () / 2 + 1); 

    for (int i = 0 ; i < covariance_matrix.rows (); ++i)
    {
      for (int j = i; j < covariance_matrix.cols (); ++j)
      {
        // Non-diagonal entries are multiplied by sqrt (2.0)
        if (j != i) 
        {
          node[k].index = k + 1;
          node[k].value = sqrt (2.0) * covariance_matrix (i, j);
          ++k;
        }
        else
        {
          node[k].index = k + 1;
          node[k].value = covariance_matrix (i, j);
          ++k;
        }
      }
    } 
    // Terminate the vector
    node[k].index = -1;
        
    double predicted_label = svm_predict (model, node);

    std::cout << "\ncluster: " << cluster_count << ", predicted label: " << predicted_label << std::endl;
    cluster_count++;

    // Color according to the class
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      colored_class_cloud->points[*pit].rgba = colors[int (predicted_label - 1)];  

    free (node);
  }   
  std::cout << "done." << std::endl;

  svm_free_and_destroy_model (&model);  
   
  // WJB 2015-07-17: This function needs to be updated to work with the latest PCL API 
  //displayPointClouds(cloud_filtered, colored_class_cloud);

  return (0);
}
