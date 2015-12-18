#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>

#define ICPNL_ptr boost::shared_ptr<pcl::IterativeClosestPointNonLinear<_PointT, _PointT> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, ICPNL, new)()
{
  return new ICPNL_ptr(new pcl::IterativeClosestPointNonLinear<_PointT, _PointT>());
}

PCLIMP(void, ICPNL, delete)(ICPNL_ptr *self)
{
  delete self;
}

PCLIMP(void, ICPNL, setInputSource)(ICPNL_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputSource(*cloud);
}

PCLIMP(void, ICPNL, setInputTarget)(ICPNL_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputTarget(*cloud);
}

PCLIMP(void, ICPNL, setMaxCorrespondenceDistance)(ICPNL_ptr *self, double distance)
{
  (*self)->setMaxCorrespondenceDistance(distance);
}

PCLIMP(void, ICPNL, setMaximumIterations)(ICPNL_ptr *self, int count)
{
  (*self)->setMaximumIterations(count);
}

PCLIMP(void, ICPNL, setTransformationEpsilon)(ICPNL_ptr *self, double epsilon)
{
  (*self)->setTransformationEpsilon(epsilon);
}

PCLIMP(void, ICPNL, setEuclideanFitnessEpsilon)(ICPNL_ptr *self, double epsilon)
{
  (*self)->setEuclideanFitnessEpsilon(epsilon);
}

PCLIMP(void, ICPNL, getFinalTransformation)(ICPNL_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix4f transformation = (*self)->getFinalTransformation();
  copyMatrix(transformation, output);
}

PCLIMP(double, ICPNL, getFitnessScore)(ICPNL_ptr *self, double max_range)
{
  return (*self)->getFitnessScore(max_range);
}

PCLIMP(void, ICPNL, align)(ICPNL_ptr *self, PointCloud_ptr *output, THFloatTensor *guess)
{
  if (!guess)
    (*self)->align(**output);
  else
    (*self)->align(**output, Tensor2Mat<4,4>(guess));
}

#undef ICPNL_ptr
#undef PointCloud_ptr
