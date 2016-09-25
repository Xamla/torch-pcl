#include "searchwrapper.h"
#include <pcl/segmentation/extract_clusters.h>

#define EuclideanClusterExtraction_ pcl::EuclideanClusterExtraction<_PointT>
#define EuclideanClusterExtraction_handle boost::shared_ptr<EuclideanClusterExtraction_>

#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define KdTree_ptr pcl::KdTreeFLANN<_PointT>::Ptr
#define Octree_ptr pcl::octree::OctreePointCloudSearch<_PointT>::Ptr

PCLIMP(EuclideanClusterExtraction_handle *, EuclideanClusterExtraction, new)() {
  return new EuclideanClusterExtraction_handle(new EuclideanClusterExtraction_());
}

PCLIMP(void, EuclideanClusterExtraction, delete)(EuclideanClusterExtraction_handle *handle) {
  delete handle;
}

PCLIMP(EuclideanClusterExtraction_ *, EuclideanClusterExtraction, EuclideanClusterExtraction_ptr)(EuclideanClusterExtraction_handle *handle) {
  return handle->get();
}

// PCLBase methods
PCLIMP(void, EuclideanClusterExtraction, setInputCloud)(EuclideanClusterExtraction_ *self, PointCloud_ptr *cloud) {
  self->setInputCloud(*cloud);
}

PCLIMP(void, EuclideanClusterExtraction, setIndices)(EuclideanClusterExtraction_ *self, Indices_ptr *indices) {
  self->setIndices(*indices);
}

PCLIMP(void, EuclideanClusterExtraction, setSearchMethod_Octree)(EuclideanClusterExtraction_ *self, Octree_ptr *octree) {
  boost::shared_ptr<OctreeSearchWrapper<_PointT> > wrapper(new OctreeSearchWrapper<_PointT>(*octree));
  self->setSearchMethod(wrapper);
}

PCLIMP(void, EuclideanClusterExtraction, setSearchMethod_KdTree)(EuclideanClusterExtraction_ *self, KdTree_ptr *kdtree) {
  boost::shared_ptr<KdTreeSearchWrapper<_PointT> > wrapper(new KdTreeSearchWrapper<_PointT>(*kdtree));
  self->setSearchMethod(wrapper);
}

PCLIMP(void, EuclideanClusterExtraction, setClusterTolerance)(EuclideanClusterExtraction_ *self, double tolerance) {
  self->setClusterTolerance(tolerance);
}

PCLIMP(double, EuclideanClusterExtraction, getClusterTolerance)(EuclideanClusterExtraction_ *self) {
  return self->getClusterTolerance();
}

PCLIMP(void, EuclideanClusterExtraction, setMinClusterSize)(EuclideanClusterExtraction_ *self, int min_cluster_size) {
  self->setMinClusterSize(min_cluster_size);
}

PCLIMP(int, EuclideanClusterExtraction, getMinClusterSize)(EuclideanClusterExtraction_ *self) {
  return self->getMinClusterSize();
}

PCLIMP(void, EuclideanClusterExtraction, setMaxClusterSize)(EuclideanClusterExtraction_ *self, int max_cluster_size) {
  self->setMaxClusterSize(max_cluster_size);
}

PCLIMP(int, EuclideanClusterExtraction, getMaxClusterSize)(EuclideanClusterExtraction_ *self) {
  return self->getMaxClusterSize();
}

PCLIMP(void, EuclideanClusterExtraction, extract)(EuclideanClusterExtraction_ *self, IndicesVector_ptr *clusters) {
  std::vector<pcl::PointIndices> _clusters;
  self->extract(_clusters);

  // copy indices
  (*clusters)->clear();
  for (std::vector<pcl::PointIndices>::const_iterator i = _clusters.begin(); i != _clusters.end(); ++i)
    (*clusters)->push_back(Indices_ptr(new std::vector<int>(i->indices)));
}

#undef EuclideanClusterExtraction_
#undef EuclideanClusterExtraction_handle
#undef PointCloud_ptr
#undef KdTree_ptr
#undef Octree_ptr
