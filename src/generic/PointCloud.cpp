#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

typedef struct { THFloatStorage* storage; uint32_t width, height, dim; } _PointsBuffer;

PCLIMP(void*, PointCloud, new)(uint32_t width, uint32_t height)
{
  return new pcl::PointCloud<_PointT>::Ptr(new pcl::PointCloud<_PointT>(width, height));
}

PCLIMP(void*, PointCloud, clone)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return new pcl::PointCloud<_PointT>::Ptr(new pcl::PointCloud<_PointT>(**self));
}

PCLIMP(void, PointCloud, delete)(pcl::PointCloud<_PointT>::Ptr *self)
{
  delete self;
}

PCLIMP(uint32_t, PointCloud, getHeaderSeq)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->header.seq;
}

PCLIMP(void, PointCloud, setHeaderSeq)(pcl::PointCloud<_PointT>::Ptr *self, uint32_t value)
{
  (*self)->header.seq = value;
}

PCLIMP(int32_t, PointCloud, getHeaderStamp_sec)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (int32_t)((*self)->header.stamp / 1000000000ull);
}

PCLIMP(int32_t, PointCloud, getHeaderStamp_nsec)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (int32_t)((*self)->header.stamp % 1000000000ull);
}

PCLIMP(void, PointCloud, setHeaderStamp)(pcl::PointCloud<_PointT>::Ptr *self, int32_t sec, int32_t nsec)
{
  (*self)->header.stamp = (uint64_t)sec*1000000000ull + (uint64_t)nsec;
}

PCLIMP(const char *, PointCloud, getHeaderFrameId)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->header.frame_id.c_str();
}

PCLIMP(void, PointCloud, setHeaderFrameId)(pcl::PointCloud<_PointT>::Ptr *self, const char *value)
{
  (*self)->header.frame_id = (value != NULL) ? value : "";
}

PCLIMP(uint32_t, PointCloud, getWidth)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->width;
}

PCLIMP(uint32_t, PointCloud, getHeight)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->height;
}

PCLIMP(bool, PointCloud, getIsDense)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->is_dense;
}

PCLIMP(void, PointCloud, setIsDense)(pcl::PointCloud<_PointT>::Ptr *self, bool value)
{
  (*self)->is_dense = value;
}

PCLIMP(_PointT&, PointCloud, at1D)(pcl::PointCloud<_PointT>::Ptr *self, int n)
{
  return (*self)->at(n);
}

PCLIMP(_PointT&, PointCloud, at2D)(pcl::PointCloud<_PointT>::Ptr *self, int column, int row)
{
  return (*self)->at(column, row);
}

PCLIMP(void, PointCloud, clear)(pcl::PointCloud<_PointT>::Ptr *self)
{
  (*self)->clear();
}

PCLIMP(void, PointCloud, reserve)(pcl::PointCloud<_PointT>::Ptr* self, size_t n)
{
  (*self)->reserve(n);
}

PCLIMP(uint32_t, PointCloud, size)(pcl::PointCloud<_PointT>::Ptr* self)
{
  return static_cast<uint32_t>((*self)->size());
}

PCLIMP(bool, PointCloud, empty)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->empty();
}

PCLIMP(bool, PointCloud, isOrganized)(pcl::PointCloud<_PointT>::Ptr *self)
{
  return (*self)->isOrganized();
}

PCLIMP(void, PointCloud, push_back)(pcl::PointCloud<_PointT>::Ptr *self, const _PointT& pt)
{
  (*self)->push_back(pt);
}

PCLIMP(void, PointCloud, insert)(pcl::PointCloud<_PointT>::Ptr *self, size_t position, size_t n, const _PointT& pt)
{
  if (n == 0)
    return;

  pcl::PointCloud<_PointT>& cloud = **self;
  pcl::PointCloud<_PointT>::iterator it;
  if (position >= cloud.size())
    it = cloud.end();
  else
    it = cloud.begin() + position;
  cloud.insert(it, n, pt);
}

PCLIMP(void, PointCloud, erase)(pcl::PointCloud<_PointT>::Ptr *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;

  pcl::PointCloud<_PointT>& cloud = **self;
  pcl::PointCloud<_PointT>::iterator b, e;
  if (begin >= cloud.size())
    b = cloud.end();
  else
    b = cloud.begin() + begin;
  if (end >= cloud.size())
    e = cloud.end();
  else
    e = cloud.begin() + end;
  cloud.erase(b, e);
}

PCLIMP(_PointsBuffer, PointCloud, points)(pcl::PointCloud<_PointT>::Ptr *self)
{
  float *ptr = reinterpret_cast<float*>(&(*self)->points[0]);
  _PointsBuffer buf;
  buf.dim = sizeof(_PointT) / sizeof(float);
  buf.storage = THFloatStorage_newWithData(ptr, (*self)->points.size() * buf.dim);
  buf.storage->flag = TH_STORAGE_REFCOUNTED;
  buf.width = (*self)->width;
  buf.height = (*self)->height;
  return buf;
}

PCLIMP(THFloatStorage*, PointCloud, sensorOrigin)(pcl::PointCloud<_PointT>::Ptr *self)
{
  float *ptr = &((*self)->sensor_origin_(0));
  THFloatStorage* storage = THFloatStorage_newWithData(ptr, 4);
  storage->flag = TH_STORAGE_REFCOUNTED;
  return storage;
}

PCLIMP(THFloatStorage*, PointCloud, sensorOrientation)(pcl::PointCloud<_PointT>::Ptr *self)
{
  float *ptr = &((*self)->sensor_orientation_.coeffs()(0));
  THFloatStorage* storage = THFloatStorage_newWithData(ptr, 4);
  storage->flag = TH_STORAGE_REFCOUNTED;
  return storage;
}

PCLIMP(void, PointCloud, add)(pcl::PointCloud<_PointT>::Ptr *self, pcl::PointCloud<_PointT>::Ptr *other)
{
  **self += **other;
}

#ifndef _PointT_WITHOUT_XYZ

PCLIMP(void, PointCloud, transform)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor *mat, pcl::PointCloud<_PointT>::Ptr *output)
{
  // check dimensionality of input matrix
  THArgCheck(mat != NULL && mat->nDimension == 2 && mat->size[0] == 4 && mat->size[1] == 4, 2, "4x4 matrix expected");

  // make sure tensor is contiguous
  mat = THFloatTensor_newContiguous(mat);

  // fill Eigen matrix
  Eigen::Map<Eigen::Matrix<float,4,4,Eigen::RowMajor> > m(THFloatTensor_data(mat));
  pcl::transformPointCloud(**self, **output, m);
  THFloatTensor_free(mat);
}

#ifdef _PointT_HAS_NORMALS

PCLIMP(void, PointCloud, transformWithNormals)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor *mat, pcl::PointCloud<_PointT>::Ptr *output)
{
  // check dimensionality of input matrix
  THArgCheck(mat != NULL && mat->nDimension == 2 && mat->size[0] == 4 && mat->size[1] == 4, 2, "4x4 matrix expected");

  // make sure tensor is contiguous
  mat = THFloatTensor_newContiguous(mat);

  // fill Eigen matrix
  Eigen::Map<Eigen::Matrix<float,4,4,Eigen::RowMajor> > m(THFloatTensor_data(mat));
  pcl::transformPointCloudWithNormals(**self, **output, m);
  THFloatTensor_free(mat);
}

#endif // _PointT_HAS_NORMALS

#ifdef _PointNormalT

PCLIMP(void, PointCloud, addNormals)(pcl::PointCloud<_PointT>::Ptr *self, pcl::PointCloud<pcl::Normal>::Ptr *normals, pcl::PointCloud<_PointNormalT>::Ptr *output)
{
  pcl::concatenateFields(**self, **normals, **output);
}

#endif // _PointNormalT

PCLIMP(void, PointCloud, computeCovarianceMatrix)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor *centroid, THFloatTensor *output)
{
  Eigen::Vector4f centroid_;
  if (centroid)
    centroid_ = Tensor2Vec4f(centroid);
  else
    pcl::compute3DCentroid(**self, centroid_);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrix(**self, centroid_, covariance);
  copyMatrix(covariance, output);
}

PCLIMP(void, PointCloud, getMinMax3D)(pcl::PointCloud<_PointT>::Ptr *self, _PointT &min, _PointT &max)
{
  pcl::getMinMax3D(**self, min, max);
}

PCLIMP(void, PointCloud, compute3DCentroid)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor *output)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(**self, centroid);
  copyMatrix(centroid, output);
}

PCLIMP(void, PointCloud, fromPCLPointCloud2)(pcl::PointCloud<_PointT>::Ptr *self, pcl::PCLPointCloud2 *msg)
{
  pcl::fromPCLPointCloud2<_PointT>(*msg, **self);
}

PCLIMP(void, PointCloud, toPCLPointCloud2)(pcl::PointCloud<_PointT>::Ptr *self, pcl::PCLPointCloud2 *msg)
{
  pcl::toPCLPointCloud2<_PointT>(**self, *msg);}

PCLIMP(int, PointCloud, readXYZfloat)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor *output)
{
  if (!self || !output)
    return -1;

  const pcl::PointCloud<_PointT>& c = **self;

  THFloatTensor_resize3d(output, c.height, c.width, 3);
  float *output_data = THFloatTensor_data(output);

  for (pcl::PointCloud<_PointT>::const_iterator i = c.begin(); i != c.end(); ++i)
  {
    const _PointT& p = *i;
    *output_data++ = p.x;
    *output_data++ = p.y;
    *output_data++ = p.z;
  }

  return 0;
}

#endif

/*PCLIMP(void*, PointCloud, fromTensor)(THTensor* tensor)
{

  // TODO

  // validate tensor

  // get float buffer
  return new pcl::PointCloud<_PointT>::Ptr(...);
}*/
