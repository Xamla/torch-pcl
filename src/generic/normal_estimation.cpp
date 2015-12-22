#include "searchwrapper.h"
#include <pcl/features/normal_3d_omp.h>

#define NormalEstimation_ptr pcl::NormalEstimationOMP<_PointT, pcl::Normal>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define OutputPointCloud_ptr pcl::PointCloud<pcl::Normal>::Ptr
#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define Octree_ptr pcl::octree::OctreePointCloudSearch<_PointT>::Ptr

PCLIMP(void*, NormalEstimation, new)()
{
  return new NormalEstimation_ptr(new pcl::NormalEstimationOMP<_PointT, pcl::Normal>());
}

PCLIMP(void, NormalEstimation, delete)(NormalEstimation_ptr *self)
{
  delete self;
}

PCLIMP(void, NormalEstimation, setInputCloud)(NormalEstimation_ptr *self, PointCloud_ptr* cloud)
{
  (*self)->setInputCloud(*cloud);
}


PCLIMP(void, NormalEstimation, setIndices)(NormalEstimation_ptr *self, Indices_ptr *indices)
{
  (*self)->setIndices(*indices);
}


PCLIMP(void, NormalEstimation, getViewPoint)(NormalEstimation_ptr *self, THFloatTensor* out_pt)
{
  float x, y, z;
  (*self)->getViewPoint(x, y, z);
  THFloatTensor_resize1d(out_pt, 4);
  THFloatTensor_set1d(out_pt, 0, x);
  THFloatTensor_set1d(out_pt, 1, y);
  THFloatTensor_set1d(out_pt, 2, z);
  THFloatTensor_set1d(out_pt, 3, 1);
}

PCLIMP(void, NormalEstimation, setViewPoint)(NormalEstimation_ptr *self, THFloatTensor* pt)
{
  Eigen::Vector3f v(Tensor2Vec3f(pt));
  (*self)->setViewPoint(v[0], v[1], v[2]);
}

PCLIMP(void, NormalEstimation, useSensorOriginAsViewPoint)(NormalEstimation_ptr *self)
{
  (*self)->useSensorOriginAsViewPoint();
}

PCLIMP(void, NormalEstimation, setSearchMethod_Octree)(NormalEstimation_ptr *self, Octree_ptr *octree)
{
  boost::shared_ptr<OctreeSearchWrapper<_PointT> > wrapper(new OctreeSearchWrapper<_PointT>(*octree));
  (*self)->setSearchMethod(wrapper);
}

PCLIMP(void, NormalEstimation, setSearchMethod_KdTree)(NormalEstimation_ptr *self, KdTree_ptr *kdtree)
{
  boost::shared_ptr<KdTreeSearchWrapper<_PointT> > wrapper(new KdTreeSearchWrapper<_PointT>(*kdtree));
  (*self)->setSearchMethod(wrapper);
}

PCLIMP(void, NormalEstimation, setKSearch)(NormalEstimation_ptr *self, int k)
{
  (*self)->setKSearch(k);
}

PCLIMP(int, NormalEstimation, getKSearch)(NormalEstimation_ptr *self)
{
  return (*self)->getKSearch();
}

PCLIMP(void, NormalEstimation, setRadiusSearch)(NormalEstimation_ptr *self, double radius)
{
  (*self)->setRadiusSearch(radius);
}

PCLIMP(double, NormalEstimation, getRadiusSearch)(NormalEstimation_ptr *self)
{
  return (*self)->getRadiusSearch();
}

PCLIMP(void, NormalEstimation, compute)(NormalEstimation_ptr *self, OutputPointCloud_ptr *output)
{
  (*self)->compute(**output);
}

PCLIMP(void, NormalEstimation, setNumberOfThreads)(NormalEstimation_ptr *self, unsigned int num_threads)
{
  (*self)->setNumberOfThreads(num_threads);
}

// non-member function
/*PCLIMP(bool, NormalEstimation, computePointNormal)(PointCloud_ptr *cloud, Indices_ptr *indices, THFloatTensor *plane_parameters, float &curvature)
{
  
  computePointNormal (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature)
  computePointNormal (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &plane_parameters, float &curvature)
                      
  Eigen:Vector4f v =Tensor2Vec4f()
  bool result = (*self)->computePointNormal(*cloud, );
  const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                          Eigen::Vector4f &plane_parameters, float &curvature
}*/

#undef NormalEstimation_ptr
#undef PointCloud_ptr
#undef OutputPointCloud_ptr
#undef KdTree_ptr
#undef Octree_ptr
