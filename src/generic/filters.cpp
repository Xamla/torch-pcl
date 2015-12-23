#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/kdtree.h>
#include "../CropSphere.h"

#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void, Filter, extractIndices)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::ExtractIndices<_PointT> f(keep_removed);
  f.setNegative(negative);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

template<typename T>
void Filter_shadowPointsT(PointCloud_ptr *input, Indices_ptr *indices, Normals_ptr *normals, T *output, 
  float threshold, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::ShadowPoints<_PointT, pcl::Normal> f(keep_removed);
  f.setNegative(negative);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setNormals(*normals);
  f.setThreshold(threshold);  // default 0.1
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

PCLIMP(void, Filter, shadowPoints_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Normals_ptr *normals, Indices_ptr *output, 
  float threshold, bool negative, Indices_ptr *removed_indices)
{
  Filter_shadowPointsT(input, indices, normals, output, threshold, negative, removed_indices);
}

PCLIMP(void, Filter, shadowPoints_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, Normals_ptr *normals, PointCloud_ptr *output, 
  float threshold, bool negative, Indices_ptr *removed_indices)
{
  Filter_shadowPointsT(input, indices, normals, output, threshold, negative, removed_indices);
}

PCLIMP(void, Filter, removeNaNFromPointCloud)(PointCloud_ptr *input, PointCloud_ptr *output, Indices_ptr *indices)
{
  std::vector<int>& _indices = **indices;
  if (output)
    pcl::removeNaNFromPointCloud(**input, **output, _indices);
  else
    pcl::removeNaNFromPointCloud(**input, _indices);    // dry run, does not modify the input point cloud
}

#ifdef _PointT_HAS_NORMALS

PCLIMP(void, Filter, removeNaNNormalsFromPointCloud)(PointCloud_ptr *input, PointCloud_ptr *output, Indices_ptr *indices)
{
  std::vector<int>& _indices = **indices;
  pcl::removeNaNNormalsFromPointCloud(**input, **output, _indices);
}

template<typename T>
void Filter_normalSpaceSamplingT(PointCloud_ptr *input, Indices_ptr *indices, Normals_ptr *normals, T *output,
  unsigned int samples, unsigned int binsx, unsigned int binsy, unsigned int binsz)
{
  pcl::NormalSpaceSampling<_PointT, pcl::Normal> f;
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setNormals(*normals);
  f.setSample(samples);
  f.setBins(binsx, binsy, binsz);
  f.filter(**output);
}

PCLIMP(void, Filter, normalSpaceSampling_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Normals_ptr *normals, Indices_ptr *output,
  unsigned int samples, unsigned int binsx, unsigned int binsy, unsigned int binsz)
{
  Filter_normalSpaceSamplingT(input, indices, normals, output, samples, binsx, binsy, binsz);
}

PCLIMP(void, Filter, normalSpaceSampling_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, Normals_ptr *normals, PointCloud_ptr *output,
  unsigned int samples, unsigned int binsx, unsigned int binsy, unsigned int binsz)
{
  Filter_normalSpaceSamplingT(input, indices, normals, output, samples, binsx, binsy, binsz);
}

PCLIMP(void, Filter, normalRefinement)(PointCloud_ptr *input, PointCloud_ptr *output, int k = 5, int max_iterations = 15, float convergence_threshold = 0.00001f)
{
  pcl::search::KdTree<_PointT> search;
  search.setInputCloud(*input);
  std::vector<std::vector<int> > k_indices;
  std::vector<std::vector<float> > k_sqr_distances;
  search.nearestKSearch(**input, std::vector<int>(), k, k_indices, k_sqr_distances);

  pcl::NormalRefinement<_PointT> f(k_indices, k_sqr_distances);
  f.setInputCloud(*input);
  f.filter(**output);
}

#endif

template<typename T> 
void Filter_frustumCullingT(PointCloud_ptr *input, Indices_ptr *indices, T *output,
  THFloatTensor *cameraPose, float hfov, float vfov, float np_dist, float fp_dist, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::FrustumCulling<_PointT> f(keep_removed);
  f.setNegative(negative);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);  
  f.setCameraPose(Tensor2Mat<4,4>(cameraPose));
  f.setHorizontalFOV(hfov);
  f.setVerticalFOV(vfov);
  f.setNearPlaneDistance(np_dist);
  f.setFarPlaneDistance(fp_dist);
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

PCLIMP(void, Filter, frustumCulling_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Indices_ptr *output,
  THFloatTensor *cameraPose, float hfov, float vfov, float np_dist, float fp_dist, bool negative, Indices_ptr *removed_indices)
{
  Filter_frustumCullingT(input, indices, output, cameraPose, hfov, vfov, np_dist, fp_dist, negative, removed_indices);
}

PCLIMP(void, Filter, frustumCulling_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output,
  THFloatTensor *cameraPose, float hfov, float vfov, float np_dist, float fp_dist, bool negative, Indices_ptr *removed_indices)
{
  Filter_frustumCullingT(input, indices, output, cameraPose, hfov, vfov, np_dist, fp_dist, negative, removed_indices);
}

template<typename T>
void Filter_passThroughT(PointCloud_ptr *input, Indices_ptr *indices, T *output, 
  const char* fieldName, float min, float max, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::PassThrough<_PointT> f(keep_removed);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setNegative(negative);
  f.setFilterFieldName(fieldName);
  f.setFilterLimits(min, max);
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

PCLIMP(void, Filter, passThrough_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Indices_ptr *output, 
  const char* fieldName, float min, float max, bool negative, Indices_ptr *removed_indices)
{
  Filter_passThroughT(input, indices, output, fieldName, min, max, negative, removed_indices);
}

PCLIMP(void, Filter, passThrough_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, 
  const char* fieldName, float min, float max, bool negative, Indices_ptr *removed_indices)
{
  Filter_passThroughT(input, indices, output, fieldName, min, max, negative, removed_indices);
}

template<typename T>
void Filter_cropBoxT(PointCloud_ptr *input, Indices_ptr *indices, T *output, 
  THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, 
  THFloatTensor *transform, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::CropBox<_PointT> f(keep_removed);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
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
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

PCLIMP(void, Filter, cropBox_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Indices_ptr *output, 
  THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, 
  THFloatTensor *transform, bool negative, Indices_ptr *removed_indices)
{
  Filter_cropBoxT(input, indices, output, min, max, rotation, translation, transform, negative, removed_indices);
}

PCLIMP(void, Filter, cropBox_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, 
  THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, 
  THFloatTensor *transform, bool negative, Indices_ptr *removed_indices)
{
  Filter_cropBoxT(input, indices, output, min, max, rotation, translation, transform, negative, removed_indices);
}

template<typename T>
void Filter_cropSphereT(PointCloud_ptr *input, Indices_ptr *indices, T *output, 
  THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  CropSphere<_PointT> f(keep_removed);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  if (center)
    f.setCenter(Tensor2Vec4f(center));
  f.setRadius(radius);
  if (transform)
    f.setTransform(Eigen::Affine3f(Tensor2Mat<4,4>(transform)));
  f.setNegative(negative);
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

PCLIMP(void, Filter, cropSphere_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Indices_ptr *output, 
  THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices_ptr *removed_indices)
{
  Filter_cropSphereT(input, indices, output, center, radius, transform, negative, removed_indices);
}

PCLIMP(void, Filter, cropSphere_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, 
  THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices_ptr *removed_indices)
{
  Filter_cropSphereT(input, indices, output, center, radius, transform, negative, removed_indices);
}

PCLIMP(void, Filter, voxelGrid)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, float lx, float ly, float lz)
{
  pcl::VoxelGrid<_PointT> f;
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setLeafSize(lx, ly, lz);
  f.filter(**output);
}

template<typename T>
void Filter_statisticalOutlierRemovalT(PointCloud_ptr *input, Indices_ptr *indices, T *output, 
  int meanK, double stddevMulThresh, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::StatisticalOutlierRemoval<_PointT> f(keep_removed);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setMeanK(meanK);
  f.setStddevMulThresh(stddevMulThresh);
  f.setNegative(negative);
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
}

PCLIMP(void, Filter, statisticalOutlierRemoval_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Indices_ptr *output, 
  int meanK, double stddevMulThresh, bool negative, Indices_ptr *removed_indices)
{
  Filter_statisticalOutlierRemovalT(input, indices, output, meanK, stddevMulThresh, negative, removed_indices);
}

PCLIMP(void, Filter, statisticalOutlierRemoval_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, 
  int meanK, double stddevMulThresh, bool negative, Indices_ptr *removed_indices)
{
  Filter_statisticalOutlierRemovalT(input, indices, output, meanK, stddevMulThresh, negative, removed_indices);
}

template<typename T>
void Filter_randomSampleT(PointCloud_ptr *input, Indices_ptr *indices, T *output, unsigned int count)
{
  pcl::RandomSample<_PointT> f;
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setSample(count);
  f.filter(**output);
}

PCLIMP(void, Filter, randomSample_Indices)(PointCloud_ptr *input, Indices_ptr *indices, Indices_ptr *output, unsigned int count)
{
  Filter_randomSampleT(input, indices, output, count);
}

PCLIMP(void, Filter, randomSample_Cloud)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, unsigned int count)
{
  Filter_randomSampleT(input, indices, output, count);
}

PCLIMP(void, Filter, medianFilter)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, int windowSize)
{
  pcl::MedianFilter<_PointT> f;
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setWindowSize(windowSize);
  f.filter(**output);
}

PCLIMP(void, Filter, radiusOutlierRemoval)(PointCloud_ptr *input, Indices_ptr *indices, PointCloud_ptr *output, double radius, int minNeighbors, bool negative, Indices_ptr *removed_indices)
{
  bool keep_removed = removed_indices && *removed_indices;
  pcl::RadiusOutlierRemoval<_PointT> f(keep_removed);
  f.setInputCloud(*input);
  if (indices && *indices)
    f.setIndices(*indices);
  f.setRadiusSearch(radius);
  f.setMinNeighborsInRadius(minNeighbors);
  f.setNegative(negative);
  f.filter(**output);
  if (keep_removed)
    **removed_indices = *f.getRemovedIndices();
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
    if (!cloud.is_dense && !pcl::isFinite(p))
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
