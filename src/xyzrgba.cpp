#define _PointT pcl::PointXYZRGBA
#define TYPE_KEY _XYZRGBA_
#include "utils.h"
#include "generic.cpp"
#include "generic/openni2.cpp"

PCLIMP(int, PointCloud, copyRGBA)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor* output)
{
  if (self == NULL || output == NULL)
    return -1;
    
  const pcl::PointCloud<_PointT>& c = **self;
  float* output_data = THFloatTensor_data(output);
  
  if (!THFloatTensor_isContiguous(output))
    return -2;
  
  if (THFloatTensor_nElement(output) < c.size() * 4)
    return -3;
  
  size_t ofs = 0;
  for (pcl::PointCloud<_PointT>::const_iterator i = c.begin(); i != c.end(); ++i)
  {
    const _PointT& p = *i;
    output_data[ofs+0] = p.r / 255.0f;
    output_data[ofs+1] = p.g / 255.0f;
    output_data[ofs+2] = p.b / 255.0f;
    output_data[ofs+3] = p.a / 255.0f;
    ofs += 4;
  }
  
  return 0;
}
