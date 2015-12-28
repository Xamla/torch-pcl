#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>

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

PCLIMP(void, ICP, addDistanceRejector)(ICP_ptr *self, float max_distance)
{
  pcl::registration::CorrespondenceRejectorDistance::Ptr p(new pcl::registration::CorrespondenceRejectorDistance());
  p->setMaximumDistance(max_distance);
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICP, addSurfaceNormalRejector)(ICP_ptr *self, float threshold = 1)
{
  pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr p(new pcl::registration::CorrespondenceRejectorSurfaceNormal());
  p->setThreshold(threshold);
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICP, addRANSACRejector)(ICP_ptr *self, double inlier_threshold = 0.05, int max_iterations = 1000)
{
  pcl::registration::CorrespondenceRejectorSampleConsensus<_PointT>::Ptr p(new pcl::registration::CorrespondenceRejectorSampleConsensus<_PointT>());
  p->setMaximumIterations(max_iterations);
  p->setInlierThreshold(inlier_threshold);
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICP, addOneToOneRejector)(ICP_ptr *self)
{
  pcl::registration::CorrespondenceRejectorOneToOne::Ptr p(new pcl::registration::CorrespondenceRejectorOneToOne());
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICP, addTrimmedRejector)(ICP_ptr *self, float overlap_ratio = 0.5, unsigned int min_correspondences = 0)
{
  pcl::registration::CorrespondenceRejectorTrimmed::Ptr p(new pcl::registration::CorrespondenceRejectorTrimmed());
  p->setOverlapRatio(overlap_ratio);
  p->setMinCorrespondences(min_correspondences);
  (*self)->addCorrespondenceRejector(p);
}

#undef ICP_ptr
#undef PointCloud_ptr
