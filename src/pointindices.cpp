#include <pcl/PointIndices.h>
#include "utils.h"

#define TYPE_KEY _
#define Indices_ptr pcl::PointIndices::Ptr

PCLIMP(Indices_ptr *, PointIndices, new)()
{
  return new Indices_ptr(new pcl::PointIndices());
}

PCLIMP(Indices_ptr *, PointIndices, clone)(Indices_ptr *self)
{
  Indices_ptr *clone = new Indices_ptr(new pcl::PointIndices());
  **clone = **self;
  return clone;
}

PCLIMP(void, PointIndices, delete)(Indices_ptr *self)
{
  delete self;
}

PCLIMP(unsigned int, PointIndices, size)(Indices_ptr *self)
{
  const std::vector<int>& indices = (*self)->indices;
  return static_cast<unsigned int>(indices.size());
}

PCLIMP(unsigned int, PointIndices, capacity)(Indices_ptr *self)
{
  const std::vector<int>& indices = (*self)->indices;
  return static_cast<unsigned int>(indices.capacity());
}

PCLIMP(void, PointIndices, reserve)(Indices_ptr *self, size_t capacity)
{
  std::vector<int>& indices = (*self)->indices;
  indices.reserve(capacity);
}

PCLIMP(void, PointIndices, viewAsTensor)(Indices_ptr *self, THIntTensor* tensor)
{
  std::vector<int>& indices = (*self)->indices;
  
  // create view into index buffer memory
  int *ptr = reinterpret_cast<int*>(&indices.front());
  THIntStorage *storage = THIntStorage_newWithData(ptr, indices.size() * sizeof(int));
  THIntStorage_clearFlag(storage, TH_STORAGE_FREEMEM | TH_STORAGE_RESIZABLE);

  // assign storage to tensor
  THIntTensor_setStorage1d(tensor, storage, 0, indices.size(), sizeof(int)); 
}

PCLIMP(void, PointIndices, copyToTensor)(Indices_ptr *self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count)
{
  const std::vector<int>& indices = (*self)->indices;
  
  // we can only copy to 1D tensors
  if (THIntTensor_nDimension(tensor) > 1)
    PCL_THROW_EXCEPTION(TorchPclException, "A 1D tensor was expected.");
    
  if (src_offset + count > indices.size())
    PCL_THROW_EXCEPTION(TorchPclException, "Invalid source range specified.");
  
  // all non 1D tensors will be resized to 1D (e.g. this allows to pass an emtpy destination tensor)
  if (THIntTensor_nDimension(tensor) != 1 || THIntTensor_size(tensor, 0) < dst_offset + count)
    THIntTensor_resize1d(tensor, dst_offset + count);

  // make sure we are dealing with a contiguous destination tensor
  THIntTensor* tensor_ = THIntTensor_newContiguous(tensor);
  
  // perform actual copy operation
  const int *source = &indices.front() + src_offset;
  std::memcpy(THIntTensor_data(tensor_), source, count * sizeof(int));
  
  // copy or free
  THIntTensor_freeCopyTo(tensor_, tensor);
}

PCLIMP(void, PointIndices, copyFromTensor)(Indices_ptr *self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count)
{
  std::vector<int>& dst = (*self)->indices;
  
  if (THIntTensor_nDimension(tensor) != 1)
    PCL_THROW_EXCEPTION(TorchPclException, "Only 1D source tensors are supported.");
  
  // ensure source range is valid
  long sz = THIntTensor_size(tensor, 0);
  if (src_offset + count > sz)
    PCL_THROW_EXCEPTION(TorchPclException, "Invalid source range specified.");
  
  // ensure destination range is valid
  if (dst_offset + count > dst.size())
    PCL_THROW_EXCEPTION(TorchPclException, "Invalid destination range specified.");
  
  // make sure we are working with contiguous memory
  tensor = THIntTensor_newContiguous(tensor);
  int* src = THIntTensor_data(tensor) + src_offset;
  std::copy(src, src + count, dst.begin() + dst_offset);
  THIntTensor_free(tensor);
}

PCLIMP(void, PointIndices, insertFromTensor)(Indices_ptr *self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count)
{
  std::vector<int>& dst = (*self)->indices;
  
  if (THIntTensor_nDimension(tensor) != 1)
    PCL_THROW_EXCEPTION(TorchPclException, "Only 1D source tensors are supported.");
  
  // ensure source range is valid
  long sz = THIntTensor_size(tensor, 0);
  if (src_offset + count > sz)
    PCL_THROW_EXCEPTION(TorchPclException, "Invalid source range specified.");
  
  // ensure destination position is valid
  if (dst_offset > dst.size())
    PCL_THROW_EXCEPTION(TorchPclException, "Invalid destination position specified.");
  
  // make sure we are working with contiguous memory
  tensor = THIntTensor_newContiguous(tensor);  
  int* src = THIntTensor_data(tensor) + src_offset;
  dst.reserve(dst_offset + count);
  dst.insert(dst.begin() + dst_offset, src, src + count);
  THIntTensor_free(tensor);
}

PCLIMP(int, PointIndices, getat)(Indices_ptr *self, size_t pos)
{
  std::vector<int>& indices = (*self)->indices;
  return indices[pos];
}

PCLIMP(void, PointIndices, setat)(Indices_ptr *self, size_t pos, int value)
{
  std::vector<int>& indices = (*self)->indices;
  indices[pos] = value;
}

PCLIMP(void, PointIndices, push_back)(Indices_ptr *self, int value)
{
  std::vector<int>& indices = (*self)->indices;
  indices.push_back(value);
}

PCLIMP(void, PointIndices, pop_back)(Indices_ptr *self)
{
  std::vector<int>& indices = (*self)->indices;
  indices.pop_back();
}

PCLIMP(void, PointIndices, clear)(Indices_ptr *self)
{
  std::vector<int>& indices = (*self)->indices;
  indices.clear();
}

PCLIMP(void, PointIndices, insert)(Indices_ptr *self, size_t pos, size_t n, int value)
{
  std::vector<int>& indices = (*self)->indices;
  std::vector<int>::iterator i;
  
  if (pos >= indices.size())
    i = indices.end();
    
  indices.insert(i, n, value);
}

PCLIMP(void, PointIndices, erase)(Indices_ptr *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;
    
  std::vector<int>& indices = (*self)->indices;
  std::vector<int>::iterator b, e;
  if (begin >= indices.size())
    b = indices.end();
  else
    b = indices.begin() + begin;
  if (end >= indices.size())
    e = indices.end();
  else
    e = indices.begin() + end;
  indices.erase(b, e);
}

PCLIMP(bool, PointIndices, empty)(Indices_ptr *self)
{
  const std::vector<int>& indices = (*self)->indices;
  return indices.empty();
}
