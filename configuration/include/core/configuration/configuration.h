/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <libconfig.h++>
#include <core/console/print.h>

// Filter range outliers
typedef struct {
  float min_distance;
  float max_distance;
} Range;

// Filter a planar model
typedef struct {
  float distance_threshold;  
} Plane;

// Filter parameters 
typedef struct {
  bool enable_range;
  bool enable_plane;
  Range range;
  Plane plane;
} Filter;

// Euclidean cluster extraction
typedef struct {
  float tolerance;
  int min_points;     
  int max_points;     
} EuclideanCluster;

// Segmentation parameters
typedef struct {
  bool enable_euclidean_cluster;
  EuclideanCluster euclidean_cluster;
} Segmentation;

// Covariance descriptor
typedef struct {
  bool position;
  bool color;   
  bool normals;
  bool Ix;
  bool Iy;
  bool Ixx;
  bool Iyy;
  bool Ixy; 
  bool Imag;                 
  bool Dx;
  bool Dy;
  bool Dmag; 
  bool principle_curvatures; 
  bool gaussian_curvature;   
  float normals_radius;       
  float curvatures_radius;    
} Covariance;

// Feature descriptors 
typedef struct {
  bool enable_covariance;
  Covariance covariance;
} Descriptor;

// CORE configuration data 
typedef struct {
  Filter filter;
  Segmentation segmentation;
  Descriptor descriptor;
} COREConfiguration;

int getConfiguration (const std::string &file_name, COREConfiguration &core_cfg); 

#endif  // CONFIGURATION_H
