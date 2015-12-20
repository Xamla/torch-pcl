#ifndef searchwrapper_h
#define searchwrapper_h

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

template<typename T>
class KdTreeSearchWrapper : public pcl::search::KdTree<T>
{
public:
  KdTreeSearchWrapper(const typename pcl::KdTreeFLANN<T>::Ptr& ptr)
  {
    this->tree_ = ptr;
  }
};

template<typename T>
class OctreeSearchWrapper : public pcl::search::Octree<T>
{
public:
  OctreeSearchWrapper(const typename pcl::octree::OctreePointCloudSearch<T>::Ptr& ptr)
    : pcl::search::Octree<T>(1000)
  {
    this->tree_ = ptr;
  }
};

#endif
