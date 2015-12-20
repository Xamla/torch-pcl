#include <pcl/kdtree/kdtree_flann.h>

#define KdTreeFLANN_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, KdTreeFLANN, new)(bool sorted)
{
  return new KdTreeFLANN_ptr(new pcl::KdTreeFLANN<_PointT>(sorted));
}

PCLIMP(void*, KdTreeFLANN, clone)(KdTreeFLANN_ptr *self)
{
  return new KdTreeFLANN_ptr(new pcl::KdTreeFLANN<_PointT>(**self));
}

PCLIMP(void, KdTreeFLANN, delete)(KdTreeFLANN_ptr *self)
{
  delete self;
}

PCLIMP(void, KdTreeFLANN, setInputCloud)(KdTreeFLANN_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(float, KdTreeFLANN, getEpsilon)(KdTreeFLANN_ptr *self)
{
  return (*self)->getEpsilon();
}

PCLIMP(void, KdTreeFLANN, setEpsilon)(KdTreeFLANN_ptr *self, float value)
{
  (*self)->setEpsilon(value);
}

PCLIMP(void, KdTreeFLANN, setMinPts)(KdTreeFLANN_ptr *self, int value)
{
  (*self)->setMinPts(value);
}

PCLIMP(int, KdTreeFLANN, getMinPts)(KdTreeFLANN_ptr *self)
{
  return (*self)->getMinPts();
}

PCLIMP(void, KdTreeFLANN, setSortedResults)(KdTreeFLANN_ptr *self, bool value)
{
  (*self)->setSortedResults(value);
}

PCLIMP(void, KdTreeFLANN, assign)(KdTreeFLANN_ptr *self, KdTreeFLANN_ptr *other)
{
  if (!other || !*other)
    return;
  (*self)->operator=(**other);
}

PCLIMP(int, KdTreeFLANN, nearestKSearch)(KdTreeFLANN_ptr *self, const _PointT &point, int k, THIntTensor *indices, THFloatTensor *squaredDistances)
{
  std::vector<int> indices_;
  std::vector<float> squaredDistances_;
  
  int found = (*self)->nearestKSearch(point, k, indices_, squaredDistances_);

  if (indices)
    vector2Tensor(indices_, indices);
    
  if (squaredDistances)
    vector2Tensor(squaredDistances_, squaredDistances);

  return found;
}

PCLIMP(int, KdTreeFLANN, radiusSearch)(KdTreeFLANN_ptr *self, const _PointT &point, double radius, THIntTensor *indices, THFloatTensor *squaredDistances, unsigned int max_nn)
{
  std::vector<int> indices_;
  std::vector<float> squaredDistances_;
  
  int found = (*self)->radiusSearch(point, radius, indices_, squaredDistances_, max_nn);

  if (indices)
    vector2Tensor(indices_, indices);
    
  if (squaredDistances)
    vector2Tensor(squaredDistances_, squaredDistances);  
  
  return found;
}

#undef ICP_ptr
#undef PointCloud_ptr
