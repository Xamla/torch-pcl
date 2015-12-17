#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#define Octree_ptr boost::shared_ptr<pcl::octree::OctreePointCloudSearch<_PointT> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, OctreePointCloudSearch, new)(double resolution)
{
  return new Octree_ptr(new pcl::octree::OctreePointCloudSearch<_PointT>(resolution));
}

PCLIMP(void, OctreePointCloudSearch, delete)(Octree_ptr *self)
{
  delete self;
}

PCLIMP(double, OctreePointCloudSearch, getResolution)(Octree_ptr *self)
{
  return (*self)->getResolution();
}

PCLIMP(double, OctreePointCloudSearch, getEpsilon)(Octree_ptr *self)
{
  return (*self)->getEpsilon();
}

PCLIMP(void, OctreePointCloudSearch, setEpsilon)(Octree_ptr *self, double value)
{
  (*self)->setEpsilon(value);
}

PCLIMP(void, OctreePointCloudSearch, setInputCloud)(Octree_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, OctreePointCloudSearch, addPointsFromInputCloud)(Octree_ptr *self)
{
  (*self)->addPointsFromInputCloud();
}

PCLIMP(void, OctreePointCloudSearch, addPointToCloud)(Octree_ptr *self, const _PointT &point, PointCloud_ptr *cloud)
{
  (*self)->addPointToCloud(point, *cloud);
}

PCLIMP(bool, OctreePointCloudSearch, isVoxelOccupiedAtPoint)(Octree_ptr *self, const _PointT &point)
{
  return (*self)->isVoxelOccupiedAtPoint(point);
}

PCLIMP(void, OctreePointCloudSearch, deleteTree)(Octree_ptr *self)
{
  (*self)->deleteTree();
}

PCLIMP(void, OctreePointCloudSearch, setMaxVoxelIndex)(Octree_ptr *self, unsigned int value)
{
  (*self)->setMaxVoxelIndex(value);
}

PCLIMP(void, OctreePointCloudSearch, setTreeDepth)(Octree_ptr *self, unsigned int value)
{
  (*self)->setTreeDepth(value);
}

PCLIMP(unsigned int, OctreePointCloudSearch, getTreeDepth)(Octree_ptr *self)
{
  return (*self)->getTreeDepth();
}

PCLIMP(unsigned int, OctreePointCloudSearch, getLeafCount)(Octree_ptr *self)
{
  return static_cast<unsigned int>((*self)->getLeafCount());
}

PCLIMP(unsigned int, OctreePointCloudSearch, getBranchCount)(Octree_ptr *self)
{
  return static_cast<unsigned int>((*self)->getBranchCount());
}

PCLIMP(int, OctreePointCloudSearch, nearestKSearch)(Octree_ptr *self, const _PointT &point, int k, THIntTensor *indices, THFloatTensor *squaredDistances)
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

PCLIMP(int, OctreePointCloudSearch, radiusSearch)(Octree_ptr *self, const _PointT &point, double radius, THIntTensor *indices, THFloatTensor *squaredDistances, unsigned int max_nn)
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

#undef Octree_ptr
#undef PointCloud_ptr 
