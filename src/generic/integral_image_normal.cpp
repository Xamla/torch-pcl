#include <pcl/features/integral_image_normal.h>

#define IntegralImageNormalEstimation_ptr pcl::IntegralImageNormalEstimation<_PointT, pcl::Normal>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define OutputPointCloud_ptr pcl::PointCloud<pcl::Normal>::Ptr

PCLIMP(IntegralImageNormalEstimation_ptr*, IntegralImageNormalEstimation, new)()
{
  return new IntegralImageNormalEstimation_ptr(new pcl::IntegralImageNormalEstimation<_PointT, pcl::Normal>());
}

PCLIMP(void, IntegralImageNormalEstimation, delete)(IntegralImageNormalEstimation_ptr *self)
{
  delete self;
}

PCLIMP(void, IntegralImageNormalEstimation, setInputCloud)(IntegralImageNormalEstimation_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, IntegralImageNormalEstimation, setRectSize)(IntegralImageNormalEstimation_ptr *self, int width, int height)
{
  (*self)->setRectSize(width, height);
}

PCLIMP(void, IntegralImageNormalEstimation, setMaxDepthChangeFactor)(IntegralImageNormalEstimation_ptr *self, float max_depth_change_factor)
{
  (*self)->setMaxDepthChangeFactor(max_depth_change_factor);
}

PCLIMP(void, IntegralImageNormalEstimation, setBorderPolicy)(IntegralImageNormalEstimation_ptr *self, int border_policy)
{
  (*self)->setBorderPolicy((pcl::IntegralImageNormalEstimation<_PointT, pcl::Normal>::BorderPolicy)border_policy);
}

PCLIMP(pcl::Normal, IntegralImageNormalEstimation, computePointNormal)(IntegralImageNormalEstimation_ptr *self, int pos_x, int pos_y, unsigned int point_index)
{
  pcl::Normal result;
  (*self)->computePointNormal(pos_x, pos_y, point_index, result);
  return result;
}

PCLIMP(pcl::Normal, IntegralImageNormalEstimation, computePointNormalMirror)(IntegralImageNormalEstimation_ptr *self, int pos_x, int pos_y, unsigned int point_index)
{
  pcl::Normal result;
  (*self)->computePointNormalMirror(pos_x, pos_y, point_index, result);
  return result;
}

PCLIMP(void, IntegralImageNormalEstimation, setNormalSmoothingSize)(IntegralImageNormalEstimation_ptr *self, float normal_smoothing_size)
{
  (*self)->setNormalSmoothingSize(normal_smoothing_size);
}

PCLIMP(void, IntegralImageNormalEstimation, setNormalEstimationMethod)(IntegralImageNormalEstimation_ptr *self, int normal_estimation_method)
{
  (*self)->setNormalEstimationMethod((pcl::IntegralImageNormalEstimation<_PointT, pcl::Normal>::NormalEstimationMethod)normal_estimation_method);
}

PCLIMP(void, IntegralImageNormalEstimation, setDepthDependentSmoothing)(IntegralImageNormalEstimation_ptr *self, bool use_depth_dependent_smoothing)
{
 (*self)->setDepthDependentSmoothing(use_depth_dependent_smoothing);
}

PCLIMP(void, IntegralImageNormalEstimation, getViewPoint)(IntegralImageNormalEstimation_ptr *self, THFloatTensor *out_pt)
{
  float x, y, z;
  (*self)->getViewPoint(x, y, z);
  THFloatTensor_resize1d(out_pt, 4);
  THFloatTensor_set1d(out_pt, 0, x);
  THFloatTensor_set1d(out_pt, 1, y);
  THFloatTensor_set1d(out_pt, 2, z);
  THFloatTensor_set1d(out_pt, 3, 1);
}

PCLIMP(void, IntegralImageNormalEstimation, setViewPoint)(IntegralImageNormalEstimation_ptr *self, THFloatTensor *pt)
{
  Eigen::Vector3f v(Tensor2Vec3f(pt));
  (*self)->setViewPoint(v[0], v[1], v[2]);
}

PCLIMP(void, IntegralImageNormalEstimation, useSensorOriginAsViewPoint)(IntegralImageNormalEstimation_ptr *self)
{
  (*self)->useSensorOriginAsViewPoint();
}

PCLIMP(void, IntegralImageNormalEstimation, compute)(IntegralImageNormalEstimation_ptr *self, OutputPointCloud_ptr *output)
{
  (*self)->compute(**output);
}

#undef NormalEstimation_ptr
#undef PointCloud_ptr
#undef OutputPointCloud_ptr
