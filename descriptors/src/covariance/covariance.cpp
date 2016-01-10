/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/descriptors/covariance/covariance.h>

int 
getFeatureCount (const Covariance &covariance)
{
  int count = 0;

  if (covariance.position)
    count = count + 3;
  if (covariance.color)
    count = count + 3;
  if (covariance.normals)
    count = count + 3;
  if (covariance.principal_curvatures)
    count = count + 2;
  if (covariance.gaussian_curvature)
    count = count + 1;

  return count;
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

  // Normalize 
  avg /= feature.size (); 
  it = feature.begin ();
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
computeCovariance (const Covariance &covariance,
                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   const std::vector<pcl::PointIndices>::const_iterator it,
                   Eigen::MatrixXd &covariance_matrix)
{
  bool have_normals = false;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr 
    tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr 
    cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr 
    principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures>);
  std::vector<double> fx, fy, fz,     // Position
                      fr, fg, fb,     // Color
                      fnx, fny, fnz,  // Normals
                      pc1, pc2,       // Principal curvatures
                      K;              // Gaussian curvature

  if (covariance.normals)
  {
    computeNormals (tree, cloud, cloud_normals, covariance.normals_radius);
    have_normals = true;
  }

  if (covariance.principal_curvatures)
  {
    if (!have_normals)
      computeNormals (tree, cloud, cloud_normals, covariance.normals_radius);
    estimatePrincipalCurvatures (tree, cloud, cloud_normals, principal_curvatures, covariance.curvatures_radius);
  }

  // Fill the feature vectors
  double max_pos = 0;
  std::vector<int>::const_iterator pit; 
  for (pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  {
    // Position
    if (covariance.position)
    {
      fx.push_back (cloud->points[*pit].x);
      fy.push_back (cloud->points[*pit].y);
      fz.push_back (cloud->points[*pit].z);
      if (abs (cloud->points[*pit].x) > max_pos)
        max_pos = abs (cloud->points[*pit].x);
      if (abs (cloud->points[*pit].y) > max_pos)
        max_pos = abs (cloud->points[*pit].y);
      if (abs (cloud->points[*pit].z) > max_pos)
        max_pos = abs (cloud->points[*pit].z);
    }

    // Color
    if (covariance.color)
    { 
      fr.push_back (double (cloud->points[*pit].r) / 255);
      fg.push_back (double (cloud->points[*pit].g) / 255);
      fb.push_back (double (cloud->points[*pit].b) / 255);
    }

    // Normals
    if (covariance.normals)
    {
      fnx.push_back (cloud_normals->points[*pit].normal[0]);
      fny.push_back (cloud_normals->points[*pit].normal[1]);
      fnz.push_back (cloud_normals->points[*pit].normal[2]);
    }
    
    // Curvatures
    if (covariance.principal_curvatures)
    {
      pc1.push_back (principal_curvatures->points[*pit].pc1);
      pc2.push_back (principal_curvatures->points[*pit].pc2);
      if (covariance.gaussian_curvature)
        K.push_back (principal_curvatures->points[*pit].pc1 * principal_curvatures->points[*pit].pc2);
    }
  }
  
  // Make sure we have an object
  if (fx.empty ()) 
  {
    CORE_WARN ("Positional features missing\n"); 
    return (-1);
  }

  Eigen::MatrixXd feature_matrix (getFeatureCount (covariance), fx.size ());

  int i = 0;
  if (covariance.position)
  {
    computeMean (feature_matrix, fx, i, max_pos);
    computeMean (feature_matrix, fy, i + 1, max_pos);
    computeMean (feature_matrix, fz, i + 2, max_pos);
    i = i + 3;
  }
  if (covariance.color)
  {
    computeMean (feature_matrix, fr, i, 0);
    computeMean (feature_matrix, fg, i + 1, 0);
    computeMean (feature_matrix, fb, i + 2, 0);
    i = i + 3;
  }
  if (covariance.normals)
  {
    computeMean (feature_matrix, fnx, i, 0);
    computeMean (feature_matrix, fny, i + 1, 0);
    computeMean (feature_matrix, fnz, i + 2, 0);
    i = i + 3;
  }
  if (covariance.principal_curvatures)
  {
    computeMean (feature_matrix, pc1, i, 0);
    computeMean (feature_matrix, pc2, i + 1, 0);
    i = i + 2;
  }
  if (covariance.gaussian_curvature)
  {
    computeMean (feature_matrix, K, i, 0);
    i = i + 1;
  }

  covariance_matrix = feature_matrix * feature_matrix.transpose () / (fx.size () - 1); 
  // Force matrix symmetry
  covariance_matrix = (covariance_matrix + covariance_matrix.transpose ()) / 2; 
  covariance_matrix = covariance_matrix.log (); 

  return (0);
}

void
writeCovariance (std::ofstream &fs, const Eigen::MatrixXd &covariance_matrix)
{
  // Output upper triangular values with the non-diagonal entries multiplied by sqrt (2.0)
  for (int i = 0; i < covariance_matrix.rows (); ++i)
  {
    for (int j = i; j < covariance_matrix.cols (); ++j)
    {
      if (j != i)
        fs << sqrt (2.0) * covariance_matrix (i, j) << " ";
      else
        fs << covariance_matrix (i, j) << " ";
    }
  }        
}
