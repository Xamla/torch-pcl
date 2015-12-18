#include <pcl/point_cloud.h>
#include <pcl/registration/incremental_registration.h>

#define IncrementalReg_ptr boost::shared_ptr<pcl::registration::IncrementalRegistration<_PointT, float> >
#define ICP_ptr boost::shared_ptr<pcl::IterativeClosestPoint<_PointT, _PointT> >
#define ICPNL_ptr boost::shared_ptr<pcl::IterativeClosestPointNonLinear<_PointT, _PointT> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, IncrementalRegistration, new)()
{
  return new IncrementalReg_ptr(new pcl::registration::IncrementalRegistration<_PointT, float>());
}

PCLIMP(void, IncrementalRegistration, delete)(IncrementalReg_ptr *self)
{
  delete self;
}

PCLIMP(void, IncrementalRegistration, setICP)(IncrementalReg_ptr *self, ICP_ptr *icp)
{
  (*self)->setRegistration(*icp);
}

PCLIMP(void, IncrementalRegistration, setICPNL)(IncrementalReg_ptr *self, ICPNL_ptr *icpnl)
{
  (*self)->setRegistration(*icpnl);
}

PCLIMP(void, IncrementalRegistration, reset)(IncrementalReg_ptr *self)
{
  (*self)->reset();
}

PCLIMP(bool, IncrementalRegistration, registerCloud)(IncrementalReg_ptr *self, PointCloud_ptr *cloud, THFloatTensor* delta_estimate)
{
  Eigen::Matrix4f _delta_estimate = Eigen::Matrix<float, 4, 4>::Identity();
  if (delta_estimate)
    _delta_estimate = Tensor2Mat<4,4>(delta_estimate);
  return (*self)->registerCloud(*cloud, _delta_estimate);
}

PCLIMP(void, IncrementalRegistration, getDeltaTransform)(IncrementalReg_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix4f m = (*self)->getDeltaTransform();
  copyMatrix(m, output);
}

PCLIMP(void, IncrementalRegistration, getAbsoluteTransform)(IncrementalReg_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix4f m = (*self)->getAbsoluteTransform();
  copyMatrix(m, output);
}

#undef IncrementalReg_ptr
#undef ICP_ptr
#undef ICPNL_ptr
#undef PointCloud_ptr
