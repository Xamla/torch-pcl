#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/radius_outlier_removal.h>

PCLIMP(void*, Filter, PassThrough)(pcl::PointCloud<_PointT>::Ptr *input, const char* fieldName, float min, float max, bool negative)
{
  pcl::PassThrough<_PointT> f;
  f.setInputCloud(*input);
  f.setFilterFieldName(fieldName);
  f.setFilterLimits(min, max);
  f.setNegative(negative);
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}

PCLIMP(void*, Filter, CropBox)(pcl::PointCloud<_PointT>::Ptr *input, THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation)
{
  pcl::CropBox<_PointT> f;
  f.setInputCloud(*input);
  f.setMin(Tensor2Vec4f(min));
  f.setMax(Tensor2Vec4f(max));
  f.setRotation(Tensor2Vec3f(rotation));
  f.setTranslation(Tensor2Vec3f(translation));
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}

PCLIMP(void*, Filter, VoxelGrid)(pcl::PointCloud<_PointT>::Ptr *input, float lx, float ly, float lz)
{
  pcl::VoxelGrid<_PointT> f;
  f.setInputCloud(*input);
  f.setLeafSize(lx, ly, lz);
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}

PCLIMP(void*, Filter, StatisticalOutlierRemoval)(pcl::PointCloud<_PointT>::Ptr *input, int meanK, double stddevMulThresh)
{
  pcl::StatisticalOutlierRemoval<_PointT> f;
  f.setInputCloud(*input);
  f.setMeanK(meanK);
  f.setStddevMulThresh(stddevMulThresh);
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}

PCLIMP(void*, Filter, RandomSample)(pcl::PointCloud<_PointT>::Ptr *input, unsigned int count)
{
  pcl::RandomSample<_PointT> f;
  f.setSample(count);
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}

PCLIMP(void*, Filter, MedianFilter)(pcl::PointCloud<_PointT>::Ptr *input, int windowSize)
{
  pcl::MedianFilter<_PointT> f;
  f.setWindowSize(windowSize);
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}

PCLIMP(void*, Filter, RadiusOutlierRemoval)(pcl::PointCloud<_PointT>::Ptr *input, double radius, int minNeighbors)
{
  pcl::RadiusOutlierRemoval<_PointT> f;
  f.setRadiusSearch(radius);
  f.setMinNeighborsInRadius(minNeighbors);
  
  pcl::PointCloud<_PointT>::Ptr filtered(new pcl::PointCloud<_PointT>());
  f.filter(*filtered);
  return new pcl::PointCloud<_PointT>::Ptr(filtered);
}
