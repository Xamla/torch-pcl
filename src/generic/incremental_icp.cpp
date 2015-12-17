#include <pcl/point_cloud.h>
#include <pcl/registration/incremental_icp.h>

#define IncrementalICP_ptr boost::shared_ptr<pcl::registration::IncrementalICP<_PointT, float> >
#define ICP_ptr boost::shared_ptr<pcl::IterativeClosestPoint<_PointT, _PointT> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, IncrementalICP, new)()
{
  return new IncrementalICP_ptr(new pcl::registration::IncrementalICP<_PointT, float>());
}

PCLIMP(void, IncrementalICP, delete)(IncrementalICP_ptr *self)
{
  delete self;
}

PCLIMP(void, IncrementalICP, setICP)(IncrementalICP_ptr *self, ICP_ptr *icp)
{
  (*self)->setICP(*icp);
}

PCLIMP(void, IncrementalICP, reset)(IncrementalICP_ptr *self)
{
  (*self)->reset();
}

PCLIMP(bool, IncrementalICP, registerCloud)(IncrementalICP_ptr *self, PointCloud_ptr *cloud, THFloatTensor* delta_estimate)
{
  Eigen::Matrix4f _delta_estimate = Eigen::Matrix<float, 4, 4>::Identity();
  if (delta_estimate)
    _delta_estimate = Tensor2Mat<4,4>(delta_estimate);
  return (*self)->registerCloud(*cloud, _delta_estimate);
}

PCLIMP(void, IncrementalICP, getDeltaTransform)(IncrementalICP_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix4f m = (*self)->getDeltaTransform();
  copyMatrix(m, output);
}

PCLIMP(void, IncrementalICP, getAbsoluteTransform)(IncrementalICP_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix4f m = (*self)->getAbsoluteTransform();
  copyMatrix(m, output);
}

#undef IncrementalICP_ptr
#undef ICP_ptr
#undef PointCloud_ptr
