/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */
 
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <core/utils/utils.h>
#include <core/filters/filter_range.h>
#include <core/filters/filter_plane.h>
#include <core/segmentation/extract_clusters.h>
#include <core/descriptors/covariance/covariance.h>
#include <core/classification/svm/svm_predict_class.h>

void
printHelp (int, char** argv)
{
  CORE_INFO ("Syntax is: %s categories_in (directory list of pcd files) covariances_out (directory path to output covariance files)\n", argv[0]);
}

int 
main (int argc, char** argv)
{
  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }
  
  std::string category_file = argv[1];
  std::string covariance_dir = argv[2];
  std::vector<std::string> categories;

  if (getCategories (category_file, categories) < 0)
    return (-1);

  omp_set_nested (1);
  #pragma omp parallel for
  for (size_t i = 0; i < categories.size (); ++i)
  {
    std::vector<std::string> point_clouds;
    if (getData (categories[i], point_clouds) < 0)
      continue;
    #pragma omp parallel for
    for (size_t ii = 0; ii < point_clouds.size (); ++ii)
    {
      std::vector<int> indices;
      Eigen::MatrixXd covariance_matrix;
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr 
        tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

      std::string point_cloud = point_clouds[ii];
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (point_cloud.c_str (), *cloud) < 0) 
      { 
        CORE_ERROR ("Could not open file '%s'! Error : %s\n", point_cloud.c_str (), strerror (errno)); 
        continue;
      }

      std::cout << "Processing: " << point_cloud.c_str () << std::endl;

      std::cout << "Filtering range outliers ... ";
      filterRangeDepth (cloud, cloud_filtered, 0.0, 1.5);
      std::cout << "done." << std::endl;
 
      std::cout << "Filtering plane model ... ";
      if (filterPlaneModel (cloud_filtered, 0.015) < 0)
      {
        CORE_ERROR ("Could not estimate a planar model for the given data\n"); 
        continue;
      }
      std::cout << "done." << std::endl;
      
      pcl::removeNaNFromPointCloud (*cloud_filtered, *cloud_filtered, indices); 
      std::cout << "Segmenting cloud ... ";
      if (euclideanClusterExtraction (tree, 
                                      cloud_filtered, cluster_indices, 
                                      0.05, 500, 75000) < 0) 
      {
        CORE_ERROR ("No cluster found\n");
        continue;
      }
      std::cout << "done." << std::endl;

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        std::cout << "Computing covariance ... ";
        if (computeCovariance (cloud_filtered, it, covariance_matrix) < 0)
          continue;
        std::cout << "done." << std::endl;
      
        std::cout << "Writing out covariance ... ";
        std::string category;
        std::string covariance_file;
        std::string covariance_path;
        boost::system::error_code error;

        category = boost::filesystem::path (point_cloud.c_str ()).parent_path ().stem ().string ();
        covariance_file = boost::filesystem::path (point_cloud.c_str ()).stem ().string () + ".cov";
        covariance_path = covariance_dir + "/" + category;
        boost::filesystem::create_directories (covariance_path, error);
        if (error) 
        {
          CORE_ERROR ("Could not create directory '%s'!\n", covariance_path.c_str ()); 
          continue;
        }
        covariance_path = covariance_path + "/" + covariance_file;
        std::ofstream fs (covariance_path.c_str (), std::ios::out);
        if (!fs.is_open ()) 
        { 
          CORE_ERROR ("Could not open file '%s'! Error : %s\n", covariance_path.c_str (), strerror (errno)); 
          continue;
        }
        // Output upper triangular values with the non-diagonal entries multiplied 
        // by sqrt (2.0).  
        for (int j = 0; j < covariance_matrix.rows (); ++j)
        {
          for (int k = j; k < covariance_matrix.cols (); ++k)
          {
            if (k != j)
	          fs << sqrt (2.0) * covariance_matrix (j, k) << " ";
	        else
	          fs << covariance_matrix (j, k) << " ";
          }
        }
        std::cout << "done." << std::endl;
        fs.close ();
      }  
    }
  }

  return (0);
}
