#include "searchwrapper.h"
#include <pcl/keypoints/sift_keypoint.h>

#define SIFTKeypoint_ptr pcl::SIFTKeypoint<_PointT, pcl::PointXYZ>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define OutputPointCloud_ptr pcl::PointCloud<pcl::PointXYZ>::Ptr

#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define Octree_ptr pcl::octree::OctreePointCloudSearch<_PointT>::Ptr

PCLIMP(void*, SIFTKeypoint, new)()
{
  return new SIFTKeypoint_ptr(new pcl::SIFTKeypoint<_PointT, pcl::PointXYZ>());
}

PCLIMP(void, SIFTKeypoint, delete)(SIFTKeypoint_ptr *self)
{
  delete self;
}

PCLIMP(void, SIFTKeypoint, setInputCloud)(SIFTKeypoint_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, SIFTKeypoint, setSearchMethod_Octree)(SIFTKeypoint_ptr *self, Octree_ptr *octree)
{
  boost::shared_ptr<OctreeSearchWrapper<_PointT> > wrapper(new OctreeSearchWrapper<_PointT>(*octree));
  (*self)->setSearchMethod(wrapper);
}

PCLIMP(void, SIFTKeypoint, setSearchMethod_KdTree)(SIFTKeypoint_ptr *self, KdTree_ptr *kdtree)
{
  boost::shared_ptr<KdTreeSearchWrapper<_PointT> > wrapper(new KdTreeSearchWrapper<_PointT>(*kdtree));
  (*self)->setSearchMethod(wrapper);
}

PCLIMP(void, SIFTKeypoint, setScales)(SIFTKeypoint_ptr *self, float min_scale, int nr_octaves, int nr_scales_per_octave)
{
  (*self)->setScales(min_scale, nr_octaves, nr_scales_per_octave);
}

PCLIMP(void, SIFTKeypoint, setMinimumContrast)(SIFTKeypoint_ptr *self, float min_contrast)
{
   (*self)->setMinimumContrast(min_contrast);
}

PCLIMP(void, SIFTKeypoint, compute)(SIFTKeypoint_ptr *self, OutputPointCloud_ptr *output)
{
  (*self)->compute(**output);
}

#undef SIFTKeypoint_ptr
#undef PointCloud_ptr
#undef OutputPointCloud_ptr
#undef KdTree_ptr
#undef Octree_ptr
