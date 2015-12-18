#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#define NormalEstimation_ptr boost::shared_ptr<pcl::NormalEstimation<_PointT, pcl::Normal> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, NormalEstimation, new)(bool sorted)
{
  return new NormalEstimation_ptr(new pcl::NormalEstimation<_PointT, pcl::Normal>());
}

PCLIMP(void, NormalEstimation, delete)(KdTreeFLANN_ptr *self)
{
  delete self;
}

#undef NormalEstimation_ptr
#undef PointCloud_ptr