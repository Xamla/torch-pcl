#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define ICP_ptr boost::shared_ptr<pcl::IterativeClosestPoint<_PointT, _PointT> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, ICP, new)()
{
#ifdef _PointT_HAS_NORMALS
  return new ICP_ptr(new pcl::IterativeClosestPointWithNormals<_PointT, _PointT>());
#else
  return new ICP_ptr(new pcl::IterativeClosestPoint<_PointT, _PointT>());
#endif
}

PCLIMP(void, ICP, delete)(ICP_ptr *self)
{
  delete self;
}

PCLIMP(void, ICP, setInputSource)(ICP_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputSource(*cloud);
}

PCLIMP(void, ICP, setInputTarget)(ICP_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputTarget(*cloud);
}

PCLIMP(void, ICP, setMaxCorrespondenceDistance)(ICP_ptr *self, double distance)
{
  (*self)->setMaxCorrespondenceDistance(distance);
}

PCLIMP(void, ICP, setMaximumIterations)(ICP_ptr *self, int count)
{
  (*self)->setMaximumIterations(count);
}

PCLIMP(void, ICP, setTransformationEpsilon)(ICP_ptr *self, double epsilon)
{
  (*self)->setTransformationEpsilon(epsilon);
}

PCLIMP(void, ICP, setEuclideanFitnessEpsilon)(ICP_ptr *self, double epsilon)
{
  (*self)->setEuclideanFitnessEpsilon(epsilon);
}

PCLIMP(void, ICP, getFinalTransformation)(ICP_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix4f transformation = (*self)->getFinalTransformation();
  copyMatrix(transformation, output);
}

PCLIMP(double, ICP, getFitnessScore)(ICP_ptr *self, double max_range)
{
  return (*self)->getFitnessScore(max_range);
}

PCLIMP(void, ICP, align)(ICP_ptr *self, PointCloud_ptr *output, THFloatTensor *guess)
{
  if (!guess)
    (*self)->align(**output);
  else
    (*self)->align(**output, Tensor2Mat<4,4>(guess));
}

#undef ICP_ptr
#undef PointCloud_ptr
