#include "searchwrapper.h"
#include <pcl/features/vfh.h>

#define VFHEstimation_ptr pcl::VFHEstimation<_PointT, pcl::Normal, pcl::VFHSignature308>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define OutputPointCloud_Ptr pcl::PointCloud<pcl::VFHSignature308>::Ptr
#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define Octree_ptr pcl::octree::OctreePointCloudSearch<_PointT>::Ptr

PCLIMP(VFHEstimation_ptr*, VFHEstimation, new)()
{
  return new VFHEstimation_ptr(new pcl::VFHEstimation<_PointT, pcl::Normal, pcl::VFHSignature308>());
}

PCLIMP(void, VFHEstimation, delete)(VFHEstimation_ptr *self)
{
  delete self;
}

PCLIMP(void, VFHEstimation, setInputCloud)(VFHEstimation_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, VFHEstimation, setInputNormals)(VFHEstimation_ptr *self, Normals_ptr *normals)
{
 (*self)->setInputNormals(*normals);
}

PCLIMP(void, VFHEstimation, setIndices)(VFHEstimation_ptr *self, Indices_ptr *indices)
{
  (*self)->setIndices(*indices);
}

PCLIMP(void, VFHEstimation, getViewPoint)(VFHEstimation_ptr *self, THFloatTensor* out_pt)
{
  float x, y, z;
  (*self)->getViewPoint(x, y, z);
  THFloatTensor_resize1d(out_pt, 4);
  THFloatTensor_set1d(out_pt, 0, x);
  THFloatTensor_set1d(out_pt, 1, y);
  THFloatTensor_set1d(out_pt, 2, z);
  THFloatTensor_set1d(out_pt, 3, 1);
}

PCLIMP(void, VFHEstimation, setViewPoint)(VFHEstimation_ptr *self, THFloatTensor *pt)
{
  Eigen::Vector3f v(Tensor2Vec3f(pt));
  (*self)->setViewPoint(v[0], v[1], v[2]);
}

PCLIMP(void, VFHEstimation, setUseGivenNormal)(VFHEstimation_ptr *self, bool use)
{
  (*self)->setUseGivenNormal(use);
}
      
PCLIMP(void, VFHEstimation, setNormalToUse)(VFHEstimation_ptr *self, THFloatTensor *normal)
{
  (*self)->setNormalToUse(Tensor2Vec3f(normal));
}

PCLIMP(void, VFHEstimation, setUseGivenCentroid)(VFHEstimation_ptr *self, bool use)
{
  (*self)->setUseGivenCentroid(use);
}
      
PCLIMP(void, VFHEstimation, setCentroidToUse)(VFHEstimation_ptr *self, THFloatTensor *centroid)
{
  (*self)->setCentroidToUse(Tensor2Vec3f(centroid));
}

PCLIMP(void, VFHEstimation, setNormalizeBins)(VFHEstimation_ptr *self, bool normalize)
{
  (*self)->setNormalizeBins(normalize);
}
      
PCLIMP(void, VFHEstimation, setNormalizeDistance)(VFHEstimation_ptr *self, bool normalize)
{
  (*self)->setNormalizeDistance(normalize);
}
       
PCLIMP(void, VFHEstimation, setFillSizeComponent)(VFHEstimation_ptr *self, bool fill_size)
{
  (*self)->setFillSizeComponent(fill_size);
}
       
PCLIMP(void, VFHEstimation, compute)(VFHEstimation_ptr *self, OutputPointCloud_Ptr *output)
{
  (*self)->compute(**output);
}

PCLIMP(void, VFHEstimation, setKSearch)(VFHEstimation_ptr *self, int k)
{
  (*self)->setKSearch(k);
}

PCLIMP(int, VFHEstimation, getKSearch)(VFHEstimation_ptr *self)
{
  return (*self)->getKSearch();
}

PCLIMP(void, VFHEstimation, setRadiusSearch)(VFHEstimation_ptr *self, double radius)
{
  (*self)->setRadiusSearch(radius);
}

PCLIMP(double, VFHEstimation, getRadiusSearch)(VFHEstimation_ptr *self)
{
  return (*self)->getRadiusSearch();
}

      
#undef FPFHEstimation_ptr
#undef InPointCloud_ptr
#undef OutPointCloud_Ptr
#undef KdTree_ptr
#undef Octree_ptr
