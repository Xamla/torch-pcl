#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>

namespace xamla
{

// Redefine pcl:IterativeClosestPointNonLinear in xamla namespace:
// The original pcl version makes the member transformation_estimation_ private but
// IterativeClosestPointNonLinearPointToPlane should inherit from IterativeClosestPointNonLinear
// in order to get point-compatibility.
// Point-to-plane distance is used whenever normals are present in this torch-pcl.
template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointNonLinear :
  public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar> {
protected:
  using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::min_number_correspondences_;
  using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_estimation_;
  using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::computeTransformation;

public:
  typedef boost::shared_ptr<IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> > Ptr;
  typedef boost::shared_ptr<const IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> > ConstPtr;

  typedef typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

  IterativeClosestPointNonLinear() {
    min_number_correspondences_ = 4;
    reg_name_ = "IterativeClosestPointNonLinear";
    transformation_estimation_.reset(new pcl::registration::TransformationEstimationLM<PointSource, PointTarget, Scalar>);
  }
};

template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointNonLinearPointToPlane :
  public IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> {
protected:
  using IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar>::reg_name_;
  using IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar>::transformation_estimation_;
  using IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar>::computeTransformation;

public:
  typedef boost::shared_ptr<IterativeClosestPointNonLinearPointToPlane<PointSource, PointTarget, Scalar> > Ptr;
  typedef boost::shared_ptr<const IterativeClosestPointNonLinearPointToPlane<PointSource, PointTarget, Scalar> > ConstPtr;

  typedef typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

  IterativeClosestPointNonLinearPointToPlane() {
    reg_name_ = "IterativeClosestPointNonLinearPointToPlane";
    transformation_estimation_.reset(new pcl::registration::TransformationEstimationPointToPlane<PointSource, PointTarget, Scalar>());
  }
};

}

#define ICPNL_ptr xamla::IterativeClosestPointNonLinear<_PointT, _PointT>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, ICPNL, new)()
{
#ifdef _PointT_HAS_NORMALS
  return new ICPNL_ptr(new xamla::IterativeClosestPointNonLinearPointToPlane<_PointT, _PointT>);
#else
  return new ICPNL_ptr(new xamla::IterativeClosestPointNonLinear<_PointT, _PointT>);
#endif
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
