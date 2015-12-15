#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include "../CropSphere.h"

#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void, Filter, removeNaNFromPointCloud)(PointCloud_ptr *input, PointCloud_ptr *output, THIntTensor *index)
{
  std::vector<int> _index;
  if (output)
    pcl::removeNaNFromPointCloud(**input, **output, _index);
  else
    pcl::removeNaNFromPointCloud(**input, _index);    // dry run, does not modify the input point cloud
  if (index)
    vector2Tensor(_index, index);
}

#ifdef _PointT_HAS_NORMALS
PCLIMP(void, Filter, removeNaNNormalsFromPointCloud)(PointCloud_ptr *input, PointCloud_ptr *output, THIntTensor *index)
{
  std::vector<int> _index;
  pcl::removeNaNNormalsFromPointCloud(**input, **output, _index);
  if (index)
    vector2Tensor(_index, index);
}
#endif

PCLIMP(void, Filter, passThrough)(PointCloud_ptr *input, PointCloud_ptr *output, const char* fieldName, float min, float max, bool negative)
{
  pcl::PassThrough<_PointT> f;
  f.setInputCloud(*input);
  f.setNegative(negative);
  f.setFilterFieldName(fieldName);
  f.setFilterLimits(min, max);
  f.filter(**output);
}

PCLIMP(void, Filter, cropBox)(PointCloud_ptr *input, PointCloud_ptr *output, THFloatTensor *min, THFloatTensor *max, 
  THFloatTensor *rotation, THFloatTensor *translation, THFloatTensor *transform, bool negative)
{
  pcl::CropBox<_PointT> f;
  f.setInputCloud(*input);
  if (min)
    f.setMin(Tensor2Vec4f(min));
  if (max)
    f.setMax(Tensor2Vec4f(max));
  if (rotation)
    f.setRotation(Tensor2Vec3f(rotation));
  if (translation)
    f.setTranslation(Tensor2Vec3f(translation));
  if (transform)
    f.setTransform(Eigen::Affine3f(Tensor2Mat<4,4>(transform)));
  f.setNegative(negative);
  f.filter(**output);
}

PCLIMP(void, Filter, cropSphere)(PointCloud_ptr *input, PointCloud_ptr *output, THFloatTensor *center, double radius, THFloatTensor *transform, bool negative)
{
  CropSphere<_PointT> f;
  f.setInputCloud(*input);
  if (center)
    f.setCenter(Tensor2Vec4f(center));
  f.setRadius(radius);
  if (transform)
    f.setTransform(Eigen::Affine3f(Tensor2Mat<4,4>(transform)));
  f.setNegative(negative);
  f.filter(**output);
}

PCLIMP(void, Filter, voxelGrid)(PointCloud_ptr *input, PointCloud_ptr *output, float lx, float ly, float lz)
{
  pcl::VoxelGrid<_PointT> f;
  f.setInputCloud(*input);
  f.setLeafSize(lx, ly, lz);
  f.filter(**output);
}

PCLIMP(void, Filter, statisticalOutlierRemoval)(PointCloud_ptr *input, PointCloud_ptr *output, int meanK, double stddevMulThresh, bool negative)
{
  pcl::StatisticalOutlierRemoval<_PointT> f;
  f.setInputCloud(*input);
  f.setMeanK(meanK);
  f.setStddevMulThresh(stddevMulThresh);
  f.setNegative(negative);
  f.filter(**output);
}

PCLIMP(void, Filter, randomSample)(PointCloud_ptr *input, PointCloud_ptr *output, unsigned int count)
{
  pcl::RandomSample<_PointT> f;
  f.setInputCloud(*input);
  f.setSample(count);
  f.filter(**output);
}

PCLIMP(void, Filter, medianFilter)(PointCloud_ptr *input, PointCloud_ptr *output, int windowSize)
{
  pcl::MedianFilter<_PointT> f;
  f.setInputCloud(*input);
  f.setWindowSize(windowSize);
  f.filter(**output);
}

PCLIMP(void, Filter, radiusOutlierRemoval)(PointCloud_ptr *input, PointCloud_ptr *output, double radius, int minNeighbors, bool negative)
{
  pcl::RadiusOutlierRemoval<_PointT> f;
  f.setInputCloud(*input);
  f.setRadiusSearch(radius);
  f.setMinNeighborsInRadius(minNeighbors);
  f.setNegative(negative);
  f.filter(**output);
}

PCLIMP(int, Filter, voxelHistogram)(PointCloud_ptr *input, THFloatTensor *output, 
  int w = 48, int h = 48, int t = 48, float voxelSize = 1, 
  float originX = 0, float originY = 0, float originZ = 0, bool center = false)
{
  if (!input || !*input)
    PCL_THROW_EXCEPTION(TorchPclException, "invalid input cloud");
  
  if (!output)
    PCL_THROW_EXCEPTION(TorchPclException, "invalid output tensor");
  
  if (w <= 0 || h <= 0 || t <= 0)
    PCL_THROW_EXCEPTION(TorchPclException, "invalid volume specified");
    
  if (voxelSize <= 0)
    PCL_THROW_EXCEPTION(TorchPclException, "voxel size must be positive");
  
  const pcl::PointCloud<_PointT>& cloud = **input;
  
  THFloatTensor_resize3d(output, w, h, t);
  THFloatTensor* output_ = THFloatTensor_newContiguous(output);
  THFloatTensor_zero(output_);
  float* data = THFloatTensor_data(output_);
  
  const size_t zstride = static_cast<size_t>(THFloatTensor_stride(output_, 0));
  const size_t ystride = static_cast<size_t>(THFloatTensor_stride(output_, 1));
  const size_t xstride = static_cast<size_t>(THFloatTensor_stride(output_, 2));

  // offsets
  Eigen::Vector4f offset(Eigen::Vector4f::Zero());
  if (center)
    offset = -Eigen::Vector4f(w, h, t, 1) * 0.5;
  int count = 0;
  for (pcl::PointCloud<_PointT>::const_iterator i = cloud.begin(); i != cloud.end(); ++i)
  {
    const _PointT& p = *i;
    if (!cloud.is_dense && !isFinite(p))
      continue;
    
    // compute index
    int x = static_cast<int>(std::floor((p.x - originX) / voxelSize + offset[0]));
    if (x < 0 || x >= w)
      continue;
    
    int y = static_cast<int>(std::floor((p.y - originY) / voxelSize + offset[1]));
    if (y < 0 || y >= h)
      continue;
      
    int z = static_cast<int>(std::floor((p.z - originZ) / voxelSize + offset[2]));
    if (z < 0 || z >= t)
      continue;
        
    data[z * zstride + y * ystride + x * xstride] += 1;
    ++count;  }
  
  THFloatTensor_freeCopyTo(output_, output);
  return count;
}

#undef PointCloud_ptr
