#include "searchwrapper.h"
#include <pcl/features/fpfh_omp.h>

#define FPFHEstimation_ptr pcl::FPFHEstimationOMP<_PointT, pcl::Normal, pcl::FPFHSignature33>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define OutputPointCloud_Ptr pcl::PointCloud<pcl::FPFHSignature33>::Ptr
#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define Octree_ptr pcl::octree::OctreePointCloudSearch<_PointT>::Ptr

PCLIMP(FPFHEstimation_ptr*, FPFHEstimation, new)()
{
  return new FPFHEstimation_ptr(new pcl::FPFHEstimationOMP<_PointT, pcl::Normal, pcl::FPFHSignature33>());
}

PCLIMP(void, FPFHEstimation, delete)(FPFHEstimation_ptr *self)
{
  delete self;
}

PCLIMP(void, FPFHEstimation, setInputCloud)(FPFHEstimation_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, FPFHEstimation, setInputNormals)(FPFHEstimation_ptr *self, Normals_ptr *normals)
{
 (*self)->setInputNormals(*normals);
}

PCLIMP(void, FPFHEstimation, setIndices)(FPFHEstimation_ptr *self, Indices_ptr *indices)
{
  (*self)->setIndices(*indices);
}

PCLIMP(void, FPFHEstimation, compute)(FPFHEstimation_ptr *self, OutputPointCloud_Ptr *output)
{
  (*self)->compute(**output);
}

PCLIMP(void, FPFHEstimation, setKSearch)(FPFHEstimation_ptr *self, int k)
{
  (*self)->setKSearch(k);
}

PCLIMP(int, FPFHEstimation, getKSearch)(FPFHEstimation_ptr *self)
{
  return (*self)->getKSearch();
}

PCLIMP(void, FPFHEstimation, setRadiusSearch)(FPFHEstimation_ptr *self, double radius)
{
  (*self)->setRadiusSearch(radius);
}

PCLIMP(double, FPFHEstimation, getRadiusSearch)(FPFHEstimation_ptr *self)
{
  return (*self)->getRadiusSearch();
}

PCLIMP(void, FPFHEstimation, setNumberOfThreads)(FPFHEstimation_ptr *self, unsigned int num_threads)
{
  (*self)->setNumberOfThreads(num_threads);
}

#undef FPFHEstimation_ptr
#undef InPointCloud_ptr
#undef OutPointCloud_Ptr
#undef KdTree_ptr
#undef Octree_ptr
