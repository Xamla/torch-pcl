#include <pcl/features/organized_edge_detection.h>

#define OrganizedEdgeBase_ pcl::OrganizedEdgeBase<_PointT, pcl::Label>
#define OrganizedEdgeBase_handle boost::shared_ptr<OrganizedEdgeBase_>

#define OrganizedEdgeFromNormals_ pcl::OrganizedEdgeFromNormals<_PointT, pcl::Normal, pcl::Label>
#define OrganizedEdgeFromNormals_handle boost::shared_ptr<OrganizedEdgeFromNormals_>

#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define OutputPointCloud_ptr pcl::PointCloud<pcl::Label>::Ptr

PCLIMP(OrganizedEdgeBase_handle *, OrganizedEdgeBase, new)()
{
  return new OrganizedEdgeBase_handle(new OrganizedEdgeBase_());
}

PCLIMP(void, OrganizedEdgeBase, delete)(OrganizedEdgeBase_handle *handle)
{
  delete handle;
}

PCLIMP(OrganizedEdgeBase_ *, OrganizedEdgeBase, OrganizedEdgeBase_ptr)(OrganizedEdgeBase_handle *handle)
{
  return handle->get();
}

// PCLBase methods
PCLIMP(void, OrganizedEdgeBase, setInputCloud)(OrganizedEdgeBase_ *self, PointCloud_ptr *cloud)
{
  self->setInputCloud(*cloud);
}

PCLIMP(void, OrganizedEdgeBase, setIndices)(OrganizedEdgeBase_ *self, Indices_ptr *indices)
{
  self->setIndices(*indices);
}

// OrganizedEdgeBase methods
PCLIMP(void, OrganizedEdgeBase, setDepthDisconThreshold)(OrganizedEdgeBase_ *self, float th)
{
  self->setDepthDisconThreshold(th);
}
      
PCLIMP(void, OrganizedEdgeBase, setMaxSearchNeighbors)(OrganizedEdgeBase_ *self, int max_dist)
{
  self->setMaxSearchNeighbors(max_dist);
}

PCLIMP(void, OrganizedEdgeBase, setEdgeType)(OrganizedEdgeBase_ *self, int edge_types)
{
  self->setEdgeType(edge_types);
}
      
PCLIMP(void, OrganizedEdgeBase, compute)(OrganizedEdgeBase_ *self, OutputPointCloud_ptr *labels, IndicesVector_ptr *label_indices)
{
  std::vector<pcl::PointIndices> _label_indices;
  self->compute(**labels, _label_indices);
  
  // copy indices
  (*label_indices)->clear();
  for (std::vector<pcl::PointIndices>::const_iterator i = _label_indices.begin(); i != _label_indices.end(); ++i)
    (*label_indices)->push_back(Indices_ptr(new std::vector<int>(i->indices)));
}

/////
// OrganizedEdgeFromNormals
/////
PCLIMP(OrganizedEdgeFromNormals_handle *, OrganizedEdgeFromNormals, new)()
{
  return new OrganizedEdgeFromNormals_handle(new OrganizedEdgeFromNormals_());
}

PCLIMP(void, OrganizedEdgeFromNormals, delete)(OrganizedEdgeFromNormals_handle *handle)
{
  delete handle;
}

PCLIMP(OrganizedEdgeBase_ *, OrganizedEdgeFromNormals, OrganizedEdgeBase_ptr)(OrganizedEdgeFromNormals_handle *handle)
{
  return static_cast<OrganizedEdgeBase_ *>(handle->get());
}

PCLIMP(OrganizedEdgeFromNormals_ *, OrganizedEdgeFromNormals, OrganizedEdgeFromNormals_ptr)(OrganizedEdgeFromNormals_handle *handle)
{
  return handle->get();
}

PCLIMP(void, OrganizedEdgeFromNormals, setInputNormals)(OrganizedEdgeFromNormals_ *self, Normals_ptr *normals)
{
  self->setInputNormals(*normals);
}

PCLIMP(void, OrganizedEdgeFromNormals, setHCCannyLowThreshold)(OrganizedEdgeFromNormals_ *self, float th)
{
  self->setHCCannyLowThreshold(th);
}

PCLIMP(void, OrganizedEdgeFromNormals, setHCCannyHighThreshold)(OrganizedEdgeFromNormals_ *self, float th)
{
  self->setHCCannyHighThreshold(th);
}

PCLIMP(void, OrganizedEdgeFromNormals, compute)(OrganizedEdgeFromNormals_ *self, OutputPointCloud_ptr *labels, IndicesVector_ptr *label_indices)
{
  std::vector<pcl::PointIndices> _label_indices;
  self->compute(**labels, _label_indices);
  
  // copy indices
  (*label_indices)->clear();
  for (std::vector<pcl::PointIndices>::const_iterator i = _label_indices.begin(); i != _label_indices.end(); ++i)
    (*label_indices)->push_back(Indices_ptr(new std::vector<int>(i->indices)));
}

/*
#ifdef _PointT_HAS_COLOR

pcl::OrganizedEdgeFromRGB<_PointT, pcl::Label>

compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

pcl::OrganizedEdgeFromRGBNormals<_PointT, pcl::Normal, pcl::Label>

compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

#endif
*/

#undef OrganizedEdgeBase_
#undef OrganizedEdgeBase_handle
#undef OrganizedEdgeFromNormals_
#undef OrganizedEdgeFromNormals_handle
#undef PointCloud_ptr
#undef OutputPointCloud_ptr
