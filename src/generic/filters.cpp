#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/radius_outlier_removal.h>

#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void, Filter, passThrough)(PointCloud_ptr *input, PointCloud_ptr *output, const char* fieldName, float min, float max, bool negative)
{
  pcl::PassThrough<_PointT> f;
  f.setInputCloud(*input);
  f.setNegative(negative);
  f.setFilterFieldName(fieldName);
  f.setFilterLimits(min, max);
  f.filter(*output);
}

PCLIMP(void, Filter, cropBox)(PointCloud_ptr *input, PointCloud_ptr *output, THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation)
{
  pcl::CropBox<_PointT> f;
  f.setInputCloud(*input);
  f.setMin(Tensor2Vec4f(min));
  f.setMax(Tensor2Vec4f(max));
  f.setRotation(Tensor2Vec3f(rotation));
  f.setTranslation(Tensor2Vec3f(translation));
  f.filter(*output);
}

PCLIMP(void, Filter, voxelGrid)(PointCloud_ptr *input, PointCloud_ptr *output, float lx, float ly, float lz)
{
  pcl::VoxelGrid<_PointT> f;
  f.setInputCloud(*input);
  f.setLeafSize(lx, ly, lz);
  f.filter(*output);
}

PCLIMP(void, Filter, statisticalOutlierRemoval)(PointCloud_ptr *input, PointCloud_ptr *output, int meanK, double stddevMulThresh, bool negative)
{
  pcl::StatisticalOutlierRemoval<_PointT> f;
  f.setInputCloud(*input);
  f.setMeanK(meanK or 2);
  f.setStddevMulThresh(stddevMulThresh or 0);
  f.setNegative(negative);
  f.filter(*output);
}

PCLIMP(void, Filter, randomSample)(PointCloud_ptr *input, PointCloud_ptr *output, unsigned int count)
{
  pcl::RandomSample<_PointT> f;
  f.setInputCloud(*input);
  f.setSample(count);
  f.filter(*output);
}

PCLIMP(void, Filter, medianFilter)(PointCloud_ptr *input, PointCloud_ptr *output, int windowSize)
{
  pcl::MedianFilter<_PointT> f;
  f.setInputCloud(*input);
  f.setWindowSize(windowSize);
  f.filter(*output);
}

PCLIMP(void, Filter, radiusOutlierRemoval)(PointCloud_ptr *input, PointCloud_ptr *output, double radius, int minNeighbors)
{
  pcl::RadiusOutlierRemoval<_PointT> f;
  f.setInputCloud(*input);
  f.setRadiusSearch(radius);
  f.setMinNeighborsInRadius(minNeighbors);
  f.filter(*output);
}

#undef PointCloud_ptr
