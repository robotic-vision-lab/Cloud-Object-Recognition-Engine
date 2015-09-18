/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/descriptors/covariance/covariance.h>

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
computeCovariance (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   const std::vector<pcl::PointIndices>::const_iterator it,
                   Eigen::MatrixXd &covariance_matrix)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr 
    tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr 
    cloud_normals (new pcl::PointCloud<pcl::Normal>);
  std::vector<double> fx, fy, fz,     // Position
                      fr, fg, fb,     // Color
                      fnx, fny, fnz;  // Normals
                      
  computeNormals (tree, cloud, cloud_normals, 0.03);

  // Fill the feature vectors
  double max_pos = 0;
  std::vector<int>::const_iterator pit; 
  for (pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  {
    // Position
    fx.push_back (cloud->points[*pit].x);
    fy.push_back (cloud->points[*pit].y);
    fz.push_back (cloud->points[*pit].z);
    if (abs (cloud->points[*pit].x) > max_pos)
      max_pos = abs (cloud->points[*pit].x);
    if (abs (cloud->points[*pit].y) > max_pos)
      max_pos = abs (cloud->points[*pit].y);
    if (abs (cloud->points[*pit].z) > max_pos)
      max_pos = abs (cloud->points[*pit].z);

    // Color
    fr.push_back (double (cloud->points[*pit].r) / 255);
    fg.push_back (double (cloud->points[*pit].g) / 255);
    fb.push_back (double (cloud->points[*pit].b) / 255);

    // Normals
    fnx.push_back (cloud_normals->points[*pit].normal[0]);
    fny.push_back (cloud_normals->points[*pit].normal[1]);
    fnz.push_back (cloud_normals->points[*pit].normal[2]);
  }
  
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
