/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#include <core/filters/filter_range.h>

void
filterRangeDepth (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
                  const float min, const float max)
{
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr
    range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, min)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, max)));

  pcl::ConditionalRemoval<pcl::PointXYZRGB> range_filter;
  range_filter.setCondition (range_cond);
  range_filter.setInputCloud (cloud);
  range_filter.setKeepOrganized (true);
  range_filter.filter (*cloud_filtered); 
}
