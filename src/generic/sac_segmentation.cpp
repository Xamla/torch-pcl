#include "searchwrapper.h"
#include <pcl/segmentation/sac_segmentation.h>

#define SACSegmentation_ pcl::SACSegmentation<_PointT>
#define SACSegmentation_handle boost::shared_ptr<SACSegmentation_>
#define SACSegmentationFromNormals_ pcl::SACSegmentationFromNormals<_PointT, pcl::Normal>
#define SACSegmentationFromNormals_handle boost::shared_ptr<SACSegmentationFromNormals_>

#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define Octree_ptr pcl::octree::OctreePointCloudSearch<_PointT>::Ptr

PCLIMP(SACSegmentation_handle *, SACSegmentation, new)()
{
  return new SACSegmentation_handle(new pcl::SACSegmentation<_PointT>());
}

PCLIMP(void, SACSegmentation, delete)(SACSegmentation_handle *handle)
{
  delete handle;
}

PCLIMP(SACSegmentation_ *, SACSegmentation, SACSegmentation_ptr)(SACSegmentation_handle *handle)
{
  return handle->get();
}

// PCLBase methods
PCLIMP(void, SACSegmentation, setInputCloud)(SACSegmentation_ *self, PointCloud_ptr *cloud)
{
  self->setInputCloud(*cloud);
}

PCLIMP(void, SACSegmentation, setIndices)(SACSegmentation_ *self, Indices_ptr *indices)
{
  self->setIndices(*indices);
}

// SACSegmentation methods
PCLIMP(void, SACSegmentation, setModelType)(SACSegmentation_ *self, int model)
{
 self->setModelType(model);
}

PCLIMP(int, SACSegmentation, getModelType)(SACSegmentation_ *self) 
{
  return self->getModelType();
}

PCLIMP(void, SACSegmentation, setDistanceThreshold)(SACSegmentation_ *self, double threshold)
{
  self->setDistanceThreshold(threshold);
}

PCLIMP(double, SACSegmentation, getDistanceThreshold)(SACSegmentation_ *self)
{
  return self->getDistanceThreshold();
}

PCLIMP(void, SACSegmentation, setMaxIterations)(SACSegmentation_ *self, int max_iterations)
{
  self->setMaxIterations(max_iterations);
}

PCLIMP(int, SACSegmentation, getMaxIterations)(SACSegmentation_ *self)
{
  return self->getMaxIterations();
}

PCLIMP(void, SACSegmentation, setProbability)(SACSegmentation_ *self, double probability)
{
  self->setProbability(probability);
}

PCLIMP(double, SACSegmentation, getProbability)(SACSegmentation_ *self)
{
  return self->getProbability();
}

PCLIMP(void, SACSegmentation, setOptimizeCoefficients)(SACSegmentation_ *self, bool optimize)
{
  self->setOptimizeCoefficients(optimize);
}

PCLIMP(bool, SACSegmentation, getOptimizeCoefficients)(SACSegmentation_ *self)
{
  return self->getOptimizeCoefficients();
}

PCLIMP(void, SACSegmentation, setSamplesMaxDist_KdTree)(SACSegmentation_ *self, double radius, KdTree_ptr *search)
{
  pcl::search::Search<_PointT>::Ptr search_(new KdTreeSearchWrapper<_PointT>(*search));
  self->setSamplesMaxDist(radius, search_);
}

PCLIMP(void, SACSegmentation, setSamplesMaxDist_Octree)(SACSegmentation_ *self, double radius, Octree_ptr *search)
{
  pcl::search::Search<_PointT>::Ptr search_(new OctreeSearchWrapper<_PointT>(*search));
  self->setSamplesMaxDist(radius, search_);
}

PCLIMP(void, SACSegmentation, setRadiusLimits)(SACSegmentation_ *self, double min_radius, double max_radius)
{
  self->setRadiusLimits(min_radius, max_radius);
}

PCLIMP(void, SACSegmentation, setAxis)(SACSegmentation_ *self, THFloatTensor *axis) 
{
  self->setAxis(Tensor2Vec3f(axis));
}

PCLIMP(void, SACSegmentation, getAxis)(SACSegmentation_ *self, THFloatTensor *result) 
{
  Eigen::Vector3f axis = self->getAxis();
  copyMatrix(axis, result);
}

PCLIMP(void, SACSegmentation, setEpsAngle)(SACSegmentation_ *self, double ea)
{
  self->setEpsAngle(ea);
}

PCLIMP(double, SACSegmentation, getEpsAngle)(SACSegmentation_ *self)
{
  return self->getEpsAngle();
}

PCLIMP(void, SACSegmentation, segment)(SACSegmentation_ *self, Indices_ptr *inliers, THFloatTensor *model_coefficients)
{
  pcl::ModelCoefficients model_coefficients_;
  pcl::PointIndices inliers_;
  self->segment(inliers_, model_coefficients_);
  if (inliers != NULL)
    **inliers = inliers_.indices;
  vector2Tensor(model_coefficients_.values, model_coefficients);
}

// SACSegmentationFromNormals
PCLIMP(SACSegmentationFromNormals_handle *, SACSegmentationFromNormals, new)()
{
  return new SACSegmentationFromNormals_handle(new pcl::SACSegmentationFromNormals<_PointT, pcl::Normal>());
}

PCLIMP(void, SACSegmentationFromNormals, delete)(SACSegmentationFromNormals_handle *handle)
{
  delete handle;
}

PCLIMP(SACSegmentationFromNormals_ *, SACSegmentationFromNormals, SACSegmentationFromNormals_ptr)(SACSegmentationFromNormals_handle *handle)
{
  return handle->get();
}

PCLIMP(SACSegmentation_ *, SACSegmentationFromNormals, SACSegmentation_ptr)(SACSegmentationFromNormals_handle *handle)
{
  return static_cast<SACSegmentation_ *>(handle->get());
}

PCLIMP(void, SACSegmentationFromNormals, setInputNormals)(SACSegmentationFromNormals_ *self, Normals_ptr *normals)
{
  self->setInputNormals(*normals);
}

PCLIMP(void, SACSegmentationFromNormals, setNormalDistanceWeight)(SACSegmentationFromNormals_ *self, double distance_weight)
{
  self->setNormalDistanceWeight(distance_weight);
}

PCLIMP(void, SACSegmentationFromNormals, setMinMaxOpeningAngle)(SACSegmentationFromNormals_ *self, double min_angle, double max_angle)
{
  self->setMinMaxOpeningAngle(min_angle, max_angle);
}

PCLIMP(void, SACSegmentationFromNormals, setDistanceFromOrigin)(SACSegmentationFromNormals_ *self, const double d)
{
  self->setDistanceFromOrigin(d);
}

#undef SACSegmentation_ 
#undef SACSegmentation_handle
#undef SACSegmentationFromNormals_
#undef SACSegmentationFromNormals_handle
#undef PointCloud_ptr
#undef KdTree_ptr
#undef Octree_ptr
