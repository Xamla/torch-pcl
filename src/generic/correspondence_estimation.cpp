#include "searchwrapper.h"
#include <pcl/registration/correspondence_estimation.h>

#define CorrespondenceEstimation_ptr pcl::registration::CorrespondenceEstimation<_PointT, _PointT>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr

PCLIMP(CorrespondenceEstimation_ptr *, CorrespondenceEstimation, new)()
{
  return new CorrespondenceEstimation_ptr(new pcl::registration::CorrespondenceEstimation<_PointT, _PointT>());
}

PCLIMP(void, CorrespondenceEstimation, delete)(CorrespondenceEstimation_ptr *self)
{
  delete self;
}

PCLIMP(void, CorrespondenceEstimation, setInputSource)(CorrespondenceEstimation_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputSource(*cloud);
}

PCLIMP(void, CorrespondenceEstimation, setInputTarget)(CorrespondenceEstimation_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputTarget(*cloud);
}

PCLIMP(void, CorrespondenceEstimation, setIndicesSource)(CorrespondenceEstimation_ptr *self, Indices_ptr *indices)
{
  (*self)->setIndicesSource(*indices);
}
  
PCLIMP(void, CorrespondenceEstimation, setIndicesTarget)(CorrespondenceEstimation_ptr *self, Indices_ptr *indices)
{
  (*self)->setIndicesTarget(*indices);
}

PCLIMP(void, CorrespondenceEstimation, setSearchMethodSource)(CorrespondenceEstimation_ptr *self, KdTree_ptr *kdtree, bool force_no_recompute = false)
{
  boost::shared_ptr<KdTreeSearchWrapper<_PointT> > wrapper(new KdTreeSearchWrapper<_PointT>(*kdtree));
  (*self)->setSearchMethodSource(wrapper, force_no_recompute);
}

PCLIMP(void, CorrespondenceEstimation, setSearchMethodTarget)(CorrespondenceEstimation_ptr *self, KdTree_ptr *kdtree, bool force_no_recompute = false)
{
  boost::shared_ptr<KdTreeSearchWrapper<_PointT> > wrapper(new KdTreeSearchWrapper<_PointT>(*kdtree));
  (*self)->setSearchMethodTarget(wrapper, force_no_recompute);
}

PCLIMP(void, CorrespondenceEstimation, determineCorrespondences)(
  CorrespondenceEstimation_ptr *self,
  Correspondences_ptr *correspondences,
  double max_distance =std::numeric_limits<double>::max()
)
{
  (*self)->determineCorrespondences(**correspondences, max_distance);
}

PCLIMP(void, CorrespondenceEstimation, determineReciprocalCorrespondences)(
  CorrespondenceEstimation_ptr *self,
  Correspondences_ptr *correspondences,
  double max_distance = std::numeric_limits<double>::max()
)
{
  (*self)->determineReciprocalCorrespondences(**correspondences, max_distance);
}

#undef CorrespondenceEstimation_ptr
#undef PointCloud_ptr
#undef KdTree_ptr
