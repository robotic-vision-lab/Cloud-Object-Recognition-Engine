/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/configuration/configuration.h>

int 
getConfiguration (const std::string &file_name, COREConfiguration &core_cfg) 
{
  libconfig::Config cfg;

  try
  {
    cfg.readFile (file_name.c_str ());
  }
  catch (const libconfig::FileIOException &fioex)
  {
    CORE_ERROR ("I/O error while reading configuration file\n");
    return (-1);
  }
  catch (const libconfig::ParseException &pex)
  {
    CORE_ERROR ("Parse error in %s:  line %d - %s\n", pex.getFile (), pex.getLine (), pex.getError ());
    return (-1);
  }
  
  // Filter parameters 
  core_cfg.filter.enable_range             = cfg.lookup ("core.filter.enable_range");
  core_cfg.filter.enable_plane             = cfg.lookup ("core.filter.enable_plane");

  core_cfg.filter.range.min_distance       = cfg.lookup ("core.filter.range.min_distance");
  core_cfg.filter.range.max_distance       = cfg.lookup ("core.filter.range.max_distance");

  core_cfg.filter.plane.distance_threshold = cfg.lookup ("core.filter.plane.distance_threshold");

  // Segmentation parameters
  core_cfg.segmentation.enable_euclidean_cluster     = cfg.lookup ("core.segmentation.enable_euclidean_cluster");

  core_cfg.segmentation.euclidean_cluster.tolerance  = cfg.lookup ("core.segmentation.euclidean_cluster.tolerance");
  core_cfg.segmentation.euclidean_cluster.min_points = cfg.lookup ("core.segmentation.euclidean_cluster.min_points");
  core_cfg.segmentation.euclidean_cluster.max_points = cfg.lookup ("core.segmentation.euclidean_cluster.max_points");

  // Feature descriptors
  core_cfg.descriptor.enable_covariance               = cfg.lookup ("core.descriptor.enable_covariance");

  core_cfg.descriptor.covariance.position             = cfg.lookup ("core.descriptor.covariance.position");
  core_cfg.descriptor.covariance.color                = cfg.lookup ("core.descriptor.covariance.color");
  core_cfg.descriptor.covariance.normals              = cfg.lookup ("core.descriptor.covariance.normals");
  core_cfg.descriptor.covariance.Ix                   = cfg.lookup ("core.descriptor.covariance.Ix");
  core_cfg.descriptor.covariance.Iy                   = cfg.lookup ("core.descriptor.covariance.Iy");
  core_cfg.descriptor.covariance.Ixx                  = cfg.lookup ("core.descriptor.covariance.Ixx");
  core_cfg.descriptor.covariance.Iyy                  = cfg.lookup ("core.descriptor.covariance.Iyy");
  core_cfg.descriptor.covariance.Ixy                  = cfg.lookup ("core.descriptor.covariance.Ixy");
  core_cfg.descriptor.covariance.Imag                 = cfg.lookup ("core.descriptor.covariance.Imag");
  core_cfg.descriptor.covariance.Dx                   = cfg.lookup ("core.descriptor.covariance.Dx");
  core_cfg.descriptor.covariance.Dy                   = cfg.lookup ("core.descriptor.covariance.Dy");
  core_cfg.descriptor.covariance.Dmag                 = cfg.lookup ("core.descriptor.covariance.Dmag");
  core_cfg.descriptor.covariance.principle_curvatures = cfg.lookup ("core.descriptor.covariance.principle_curvatures");
  core_cfg.descriptor.covariance.gaussian_curvature   = cfg.lookup ("core.descriptor.covariance.gaussian_curvature");
  core_cfg.descriptor.covariance.normals_radius       = cfg.lookup ("core.descriptor.covariance.normals_radius");
  core_cfg.descriptor.covariance.curvatures_radius    = cfg.lookup ("core.descriptor.covariance.curvatures_radius");

  // Classification parameters
  core_cfg.classification.enable_svm = cfg.lookup ("core.classification.enable_svm");
  core_cfg.classification.enable_dl  = cfg.lookup ("core.classification.enable_dl");

  core_cfg.classification.svm.gamma  = cfg.lookup ("core.classification.svm.gamma");
  core_cfg.classification.svm.model  = cfg.lookup ("core.classification.svm.model").c_str ();

  core_cfg.classification.dl.atoms   = cfg.lookup ("core.classification.dl.atoms");

  return (0);
}
