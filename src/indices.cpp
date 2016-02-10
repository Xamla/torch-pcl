#include "utils.h"

#define TYPE_KEY _

PCLIMP(Indices_ptr *, Indices, new)()
{
  return new Indices_ptr(new std::vector<int>);
}

PCLIMP(Indices_ptr *, Indices, clone)(Indices_ptr *self)
{
  const std::vector<int>& indices = **self;
  return new Indices_ptr(new std::vector<int>(indices));
}

PCLIMP(void, Indices, delete)(Indices_ptr *self)
{
  delete self;
}

PCLIMP(Indices_ptr *, Indices, fromPtr)(Indices_ptr *existing)
{
  return new Indices_ptr(*existing);
}

PCLIMP(unsigned int, Indices, size)(Indices_ptr *self)
{
  const std::vector<int>& indices = **self;
  return static_cast<unsigned int>(indices.size());
}

PCLIMP(unsigned int, Indices, capacity)(Indices_ptr *self)
{
  const std::vector<int>& indices = **self;
  return static_cast<unsigned int>(indices.capacity());
}

PCLIMP(void, Indices, reserve)(Indices_ptr *self, size_t capacity)
{
  std::vector<int>& indices = **self;
  indices.reserve(capacity);
}

PCLIMP(void, Indices, append)(Indices_ptr *self, Indices_ptr *source)
{
  std::vector<int>& dst = **self;
  const std::vector<int>& src = **source;
  dst.reserve(src.size() + dst.size());
  dst.insert(dst.end(), src.begin(), src.end());
}

PCLIMP(void, Indices, insertMany)(Indices_ptr *self, Indices_ptr *source, size_t src_offset, size_t dst_offset, size_t count)
{
  std::vector<int>& dst = **self;
  const std::vector<int>& src = **source;
  
  // check if source range is valid
  if (src_offset + count > src.size())
    PCL_THROW_EXCEPTION(TorchPclException, "Invalid source range specified.")

  dst.reserve(src.size() + count);
  dst.insert(dst.begin() + dst_offset, src.begin() + src_offset, src.begin() + src_offset + count);
}

PCLIMP(void, Indices, viewAsTensor)(Indices_ptr *self, THIntTensor* tensor)
{
  std::vector<int>& indices = **self;
  
  // create view into index buffer memory
  int *ptr = &indices.front();
  THIntStorage *storage = THIntStorage_newWithData(ptr, indices.size());
  THIntStorage_clearFlag(storage, TH_STORAGE_FREEMEM | TH_STORAGE_RESIZABLE);

  // assign storage to tensor
  THIntTensor_setStorage1d(tensor, storage, 0, indices.size(), 1); 
}

PCLIMP(void, Indices, copyToTensor)(Indices_ptr *self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count)
{
  const std::vector<int>& indices = **self;
  
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

PCLIMP(void, Indices, copyFromTensor)(Indices_ptr *self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count)
{
  std::vector<int>& dst = **self;
  
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

PCLIMP(void, Indices, insertFromTensor)(Indices_ptr *self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count)
{
  std::vector<int>& dst = **self;
  
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

PCLIMP(int, Indices, getAt)(Indices_ptr *self, size_t pos)
{
  std::vector<int>& indices = **self;
  return indices[pos];
}

PCLIMP(void, Indices, setAt)(Indices_ptr *self, size_t pos, int value)
{
  std::vector<int>& indices = **self;
  indices[pos] = value;
}

PCLIMP(void, Indices, push_back)(Indices_ptr *self, int value)
{
  std::vector<int>& indices = **self;
  indices.push_back(value);
}

PCLIMP(int, Indices, pop_back)(Indices_ptr *self)
{
  std::vector<int>& indices = **self;
  int v = indices.back();
  indices.pop_back();
  return v;
}

PCLIMP(void, Indices, clear)(Indices_ptr *self)
{
  std::vector<int>& indices = **self;
  indices.clear();
}

PCLIMP(void, Indices, insert)(Indices_ptr *self, size_t pos, size_t n, int value)
{
  std::vector<int>& indices = **self;
  std::vector<int>::iterator i = pos >= indices.size() ? indices.end() : indices.begin() + pos;
  indices.insert(i, n, value);
}

PCLIMP(void, Indices, erase)(Indices_ptr *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;
    
  std::vector<int>& indices = **self;
  std::vector<int>::iterator b = begin >= indices.size() ? indices.end() : indices.begin() + begin;
  std::vector<int>::iterator e = end >= indices.size() ? indices.end() : indices.begin() + end;
  indices.erase(b, e);
}

PCLIMP(bool, Indices, empty)(Indices_ptr *self)
{
  const std::vector<int>& indices = **self;
  return indices.empty();
}
