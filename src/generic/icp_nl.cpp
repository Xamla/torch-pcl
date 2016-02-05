#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>

#define ICPNL_ptr pcl::IterativeClosestPointNonLinear<_PointT, _PointT>::Ptr
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

PCLIMP(void, ICPNL, addDistanceRejector)(ICPNL_ptr *self, float max_distance)
{
  pcl::registration::CorrespondenceRejectorDistance::Ptr p(new pcl::registration::CorrespondenceRejectorDistance());
  p->setMaximumDistance(max_distance);
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICPNL, addSurfaceNormalRejector)(ICPNL_ptr *self, float threshold = 1)
{
  pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr p(new pcl::registration::CorrespondenceRejectorSurfaceNormal());
  p->setThreshold(threshold);
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICPNL, addRANSACRejector)(ICPNL_ptr *self, double inlier_threshold = 0.05, int max_iterations = 1000)
{
  pcl::registration::CorrespondenceRejectorSampleConsensus<_PointT>::Ptr p(new pcl::registration::CorrespondenceRejectorSampleConsensus<_PointT>());
  p->setMaximumIterations(max_iterations);
  p->setInlierThreshold(inlier_threshold);
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICPNL, addOneToOneRejector)(ICPNL_ptr *self)
{
  pcl::registration::CorrespondenceRejectorOneToOne::Ptr p(new pcl::registration::CorrespondenceRejectorOneToOne());
  (*self)->addCorrespondenceRejector(p);
}

PCLIMP(void, ICPNL, addTrimmedRejector)(ICPNL_ptr *self, float overlap_ratio = 0.5, unsigned int min_correspondences = 0)
{
  pcl::registration::CorrespondenceRejectorTrimmed::Ptr p(new pcl::registration::CorrespondenceRejectorTrimmed());
  p->setOverlapRatio(overlap_ratio);
  p->setMinCorrespondences(min_correspondences);
  (*self)->addCorrespondenceRejector(p);
}

#undef ICPNL_ptr
#undef PointCloud_ptr
