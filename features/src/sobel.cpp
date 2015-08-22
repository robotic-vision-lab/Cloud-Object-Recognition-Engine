/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/features/sobel.h>

void
computeFeatureImage (const std::string input_image, 
                     const std::string input_depth,
                     cv::Mat &feature_image)
{
  int scale = 1;
  int delta = 0;
  int ddepth = CV_32F;
  cv::Mat color_src, gray_src, depth_src;

  color_src = cv::imread (input_image);
  depth_src = cv::imread (input_depth);

  color_src.convertTo (color_src, CV_32FC3);  
  std::vector<cv::Mat> bgr_planes;
  cv::split (color_src, bgr_planes);

  // Blur images and convert to grayscale
  cv::GaussianBlur (depth_src, depth_src, cv::Size (5,5), 3, 3, cv::BORDER_DEFAULT);
  depth_src.convertTo (depth_src, CV_32F); 
 
  cv::GaussianBlur (color_src, color_src, cv::Size (5,5), 3, 3, cv::BORDER_DEFAULT);
  cv::cvtColor (color_src, gray_src, CV_RGB2GRAY);
  gray_src.convertTo (gray_src, CV_32F);

  // Initialize matrices for Ix, Iy, Ixx, Ixy, Iyy, Dx, Dy, Dmag 
  cv::Mat Dx (depth_src.rows, depth_src.cols, CV_32F);
  cv::Mat Dy (depth_src.rows, depth_src.cols, CV_32F);
  cv::Mat Dmag (depth_src.rows, depth_src.cols, CV_32F);
  cv::Mat Ix (gray_src.rows, gray_src.cols, CV_32F);
  cv::Mat Iy (gray_src.rows, gray_src.cols, CV_32F);
  cv::Mat Ixx (gray_src.rows, gray_src.cols, CV_32F);
  cv::Mat Ixy (gray_src.rows, gray_src.cols, CV_32F);
  cv::Mat Iyy (gray_src.rows, gray_src.cols, CV_32F);
  cv::Mat Imag (gray_src.rows, gray_src.cols, CV_32F);

  cv::Sobel (gray_src, Ix, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel (gray_src, Iy, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

  cv::Sobel (Ix, Ixx, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel (Ix, Ixy, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel (Iy, Iyy, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

  cv::Sobel (depth_src, Dx, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel (depth_src, Dy, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  
  Imag = (Ix.mul (Ix) + Iy.mul (Iy));
  sqrt (Imag, Imag);

  Dmag = (Dx.mul (Dx) + Dy.mul (Dy));
  sqrt (Dmag, Dmag);
  
  // Build feature image 
  std::vector<cv::Mat> fi;
  fi.push_back (bgr_planes[2]);  // R
  fi.push_back (bgr_planes[1]);  // G
  fi.push_back (bgr_planes[0]);  // B
  fi.push_back (Ix);
  fi.push_back (Iy);
  fi.push_back (Ixx);
  fi.push_back (Ixy);
  fi.push_back (Iyy);
  fi.push_back (Imag);
  fi.push_back (Dx);
  fi.push_back (Dy);
  fi.push_back (Dmag);

  feature_image.create (gray_src.rows, gray_src.cols, CV_32FC (12));
  cv::merge (fi, feature_image);
}
