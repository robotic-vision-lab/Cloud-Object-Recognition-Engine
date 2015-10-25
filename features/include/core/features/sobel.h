/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef SOBEL_H
#define SOBEL_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void computeFeatureImage (const std::string input_image,
                          const std::string input_depth, 
                          cv::Mat &feature_image);

#endif  // SOBEL_H
