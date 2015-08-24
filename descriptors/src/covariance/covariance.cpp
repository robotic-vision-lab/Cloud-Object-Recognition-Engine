/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/descriptors/covariance/covariance.h>

// For timing measurements
#if 0
struct timeval start, end;
gettimeofday (&start, NULL);
gettimeofday (&end, NULL);
fprintf (stdout, "\nwall time = %.2f ms\n",
        (end.tv_sec-start.tv_sec)*1000.0
         + (end.tv_usec - start.tv_usec) / 1000.0);
#endif

void
replaceExt (std::string &s, const std::string &ext) 
{
  std::string::size_type i = s.rfind ('.', s.length ());

  if (i != std::string::npos) 
  {
    s.replace (i + 1, ext.length (), ext);
  }
}

int 
getCategories (const std::string file_name, std::vector<std::string> &categories)
{
  std::string line;
  std::ifstream fs;

  fs.open (file_name.c_str (), std::ios::in);
  if (!fs.is_open () || fs.fail ())
  {
      CORE_ERROR ("Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror (errno));
      fs.close ();
      return (-1);
  }

  while (!fs.eof ())
  {
    getline (fs, line);
    if (line == "")
      continue;
    categories.push_back (line);
  } 

  fs.close ();

  return (0);
}

int 
getPointClouds (const std::string dir_name, std::vector<std::string> &point_clouds)
{
  DIR* dp;
  struct dirent* dirp;

  if ((dp = opendir (dir_name.c_str ())) == NULL) 
  {
    CORE_ERROR ("Could not open directory '%s'! Error : %s\n", dir_name.c_str (), strerror (errno));
    return (-1);
  }

  while ((dirp = readdir (dp)) != NULL) 
  { 
    if ((strcmp (dirp->d_name, ".") == 0) || (strcmp (dirp->d_name, "..") == 0))
      continue;
    point_clouds.push_back (dir_name + "/" + dirp->d_name);
  }

  closedir (dp);

  return (0);
}

void
computeFeatures (std::vector<double> &fx, std::vector<double> &fy, std::vector<double> &fz,
                 std::vector<double> &fr, std::vector<double> &fg, std::vector<double> &fb,
                 std::vector<double> &fnx, std::vector<double> &fny, std::vector<double> &fnz,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                 const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                 const std::vector<pcl::PointIndices> &cluster_indices,
                 const std::vector<pcl::PointIndices>::const_iterator it,
                 double* max_pos)
{
  std::vector<int>::const_iterator pit; 

  *max_pos = 0;
  
  // Fill the feature vectors
  for (pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  {
    if ((isnan (normals_cloud->points[*pit].normal[0])) || 
        (isnan (normals_cloud->points[*pit].normal[1])) ||
        (isnan (normals_cloud->points[*pit].normal[2])))
      continue;
      
    // Position
    if (abs (cloud_filtered->points[*pit].x) > *max_pos)
      *max_pos = abs (cloud_filtered->points[*pit].x);
    fx.push_back (cloud_filtered->points[*pit].x);
     
    if (abs (cloud_filtered->points[*pit].y) > *max_pos)
      *max_pos = abs (cloud_filtered->points[*pit].y);
    fy.push_back (cloud_filtered->points[*pit].y);
    
    if (abs (cloud_filtered->points[*pit].z) > *max_pos)
      *max_pos = abs (cloud_filtered->points[*pit].z);
    fz.push_back (cloud_filtered->points[*pit].z);

    // Color
    fr.push_back (double (cloud_filtered->points[*pit].r) / 255);
    fg.push_back (double (cloud_filtered->points[*pit].g) / 255);
    fb.push_back (double (cloud_filtered->points[*pit].b) / 255);

    // Normals
    fnx.push_back (normals_cloud->points[*pit].normal[0]);
    fny.push_back (normals_cloud->points[*pit].normal[1]);
    fnz.push_back (normals_cloud->points[*pit].normal[2]);
  }
}

void
computeFeatures (std::vector<double> &fx, std::vector<double> &fy, std::vector<double> &fz,
                 std::vector<double> &fr, std::vector<double> &fg, std::vector<double> &fb,
                 std::vector<double> &fnx, std::vector<double> &fny, std::vector<double> &fnz,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                 const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                 const std::vector<pcl::PointIndices> &cluster_indices,
                 const int offset, double* max_pos)
{
  std::vector<int>::const_iterator pit; 
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin () + offset;

  *max_pos = 0;
  
  // Fill the feature vectors
  for (pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  {
    if ((isnan (normals_cloud->points[*pit].normal[0])) || 
        (isnan (normals_cloud->points[*pit].normal[1])) ||
        (isnan (normals_cloud->points[*pit].normal[2])))
      continue;
      
    // Position
    if (abs (cloud_filtered->points[*pit].x) > *max_pos)
      *max_pos = abs (cloud_filtered->points[*pit].x);
    fx.push_back (cloud_filtered->points[*pit].x);
     
    if (abs (cloud_filtered->points[*pit].y) > *max_pos)
      *max_pos = abs (cloud_filtered->points[*pit].y);
    fy.push_back (cloud_filtered->points[*pit].y);
    
    if (abs (cloud_filtered->points[*pit].z) > *max_pos)
      *max_pos = abs (cloud_filtered->points[*pit].z);
    fz.push_back (cloud_filtered->points[*pit].z);

    // Color
    fr.push_back (double (cloud_filtered->points[*pit].r) / 255);
    fg.push_back (double (cloud_filtered->points[*pit].g) / 255);
    fb.push_back (double (cloud_filtered->points[*pit].b) / 255);

    // Normals
    fnx.push_back (normals_cloud->points[*pit].normal[0]);
    fny.push_back (normals_cloud->points[*pit].normal[1]);
    fnz.push_back (normals_cloud->points[*pit].normal[2]);
  }
}

void
computeMean (Eigen::MatrixXd &F, const std::vector<double> &feature, 
            const int idx, const double max_pos)
{
  size_t i;
  double avg = 0;
  std::vector<double>::const_iterator it;

  for (it = feature.begin (); it != feature.end (); ++it)
    avg += *it; 
  avg /= feature.size (); 
  it = feature.begin ();
  // Normalize 
  #pragma omp parallel for
  for (i = 0; i < feature.size (); ++i) 
  {
    if (max_pos) 
      F (idx, i) = (*(it + i) - avg) / max_pos; 
    else 
      F (idx, i) = (*(it + i) - avg); 
  }
}

int
computeCovariance (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                   const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                   const std::vector<pcl::PointIndices> &cluster_indices,
                   const std::vector<pcl::PointIndices>::const_iterator it,
                   Eigen::MatrixXd &covariance_matrix)
{
  double max_pos;
  std::vector<double> fx, fy, fz,    // Position
                      fr, fg, fb,    // Color
                      fnx, fny, fnz; // Normals
                    
  computeFeatures (fx, fy, fz, fr, fg, fb, fnx, fny, fnz,
                   cloud_filtered, normals_cloud, cluster_indices,
                   it, &max_pos);

  if (fx.empty ()) 
  { 
    CORE_WARN ("Positional features missing\n"); 
    return (-1);
  }
  
  Eigen::MatrixXd feature_matrix (9, fx.size ());

  computeMean (feature_matrix, fx, 0, max_pos);
  computeMean (feature_matrix, fy, 1, max_pos);
  computeMean (feature_matrix, fz, 2, max_pos);
  computeMean (feature_matrix, fr, 3, 0);
  computeMean (feature_matrix, fg, 4, 0);
  computeMean (feature_matrix, fb, 5, 0);
  computeMean (feature_matrix, fnx, 6, 0);
  computeMean (feature_matrix, fny, 7, 0);
  computeMean (feature_matrix, fnz, 8, 0);
 
  covariance_matrix = feature_matrix * feature_matrix.transpose () / (fx.size () - 1); 
  // Force matrix symmetry
  covariance_matrix = (covariance_matrix + covariance_matrix.transpose ()) / 2; 
  covariance_matrix = covariance_matrix.log (); 

  return (0);
}

int 
computeCovariances (const std::string category_file, const std::string covariance_dir)
{
  std::vector<std::string> categories;

  if (getCategories (category_file, categories) < 0)
    return (-1);
  omp_set_nested (1);
  #pragma omp parallel for
  for (size_t i = 0; i < categories.size (); ++i)
  {
    std::vector<std::string> point_clouds;
    if (getPointClouds (categories[i], point_clouds) < 0)
      continue;
    #pragma omp parallel for
    for (size_t ii = 0; ii < point_clouds.size (); ++ii)
    {
      int offset;
      double max_pos;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr 
        tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::Normal>::Ptr 
        normals_cloud (new pcl::PointCloud<pcl::Normal>);
      std::vector<pcl::PointIndices> cluster_indices;
      std::vector<double> fx, fy, fz,     // Position
                          fr, fg, fb,     // Color
                          fnx, fny, fnz;  // Normals

      std::string point_cloud = point_clouds[ii];

      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (point_cloud.c_str (), *cloud) < 0) 
      { 
        CORE_ERROR ("Could not open file '%s'! Error : %s\n", point_cloud.c_str (), strerror (errno)); 
        continue;
      }

      std::cout << "processing: " << point_cloud.c_str () << std::endl;

      std::cout << "Filtering range outliers ... ";
      filterDepthRange (cloud, cloud_filtered, 0.0, 1.5);
      std::cout << "done." << std::endl;
 
      std::cout << "Filtering plane model ... ";
      if (filterPlaneModel (cloud_filtered, 0.015) < 0)
      {
        CORE_ERROR ("Could not estimate a planar model for the given data\n"); 
        continue;
      }
      std::cout << "done." << std::endl;

      std::cout << "Computing normals ... ";
      computeIntegralImageNormals (cloud_filtered, normals_cloud, 0.02, 10.0);
      std::cout << "done." << std::endl;

      std::cout << "Segmenting cloud ... ";
      if (euclideanClusterExtraction (tree, 
                                      cloud_filtered, cluster_indices, 
                                      0.05, 500, 75000, &offset) < 0) 
      {
        CORE_ERROR ("No cluster found\n");
        continue;
      }
      std::cout << "done." << std::endl;
      
      std::cout << "Computing features ... ";
      computeFeatures (fx, fy, fz, fr, fg, fb, fnx, fny, fnz,
                       cloud_filtered, normals_cloud, cluster_indices,
                       offset, &max_pos);
      std::cout << "done." << std::endl;

      if (fx.empty ()) 
      {
        CORE_WARN ("Positional features missing\n"); 
        continue;
      }

      Eigen::MatrixXd feature_matrix (9, fx.size ());

      std::cout << "Computing means ... ";
      computeMean (feature_matrix, fx, 0, max_pos);
      computeMean (feature_matrix, fy, 1, max_pos);
      computeMean (feature_matrix, fz, 2, max_pos);
      computeMean (feature_matrix, fr, 3, 0);
      computeMean (feature_matrix, fg, 4, 0);
      computeMean (feature_matrix, fb, 5, 0);
      computeMean (feature_matrix, fnx, 6, 0);
      computeMean (feature_matrix, fny, 7, 0);
      computeMean (feature_matrix, fnz, 8, 0);
      std::cout << "done." << std::endl;
 
      std::cout << "Computing covariance ... ";
      Eigen::MatrixXd covariance_matrix = feature_matrix * feature_matrix.transpose () / (fx.size () - 1); 
      //  Force matrix symmetry
      covariance_matrix = (covariance_matrix + covariance_matrix.transpose ()) / 2; 
      covariance_matrix = covariance_matrix.log (); 
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

  return (0);
}
