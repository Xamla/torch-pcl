#define _PointT pcl::PointXYZRGBA
#define TYPE_KEY _XYZRGBA_
#include "utils.h"
#include "generic.cpp"
#include "generic/openni2.cpp"

PCLIMP(int, PointCloud, readRGBAbyte)(pcl::PointCloud<_PointT>::Ptr *self, THByteTensor* output)
{
  if (!self || !output)
    return -1;
    
  const pcl::PointCloud<_PointT>& c = **self;

  if (!THByteTensor_isContiguous(output))
    return -2;
  
  THByteTensor_resize3d(output, c.height, c.width, 4);
  
  unsigned char *output_data = THByteTensor_data(output);
  for (pcl::PointCloud<_PointT>::const_iterator i = c.begin(); i != c.end(); ++i)
  {
    const _PointT& p = *i;
    *output_data++ = p.r;
    *output_data++ = p.g;
    *output_data++ = p.b;
    *output_data++ = p.a;
  }
  
  return 0;
}

PCLIMP(int, PointCloud, readRGBAfloat)(pcl::PointCloud<_PointT>::Ptr *self, THFloatTensor* output)
{
  if (!self || !output)
    return -1;
    
  const pcl::PointCloud<_PointT>& c = **self;
  
  if (!THFloatTensor_isContiguous(output))
    return -2;

  THFloatTensor_resize3d(output, c.height, c.width, 4);
  
  float *output_data = THFloatTensor_data(output);
  for (pcl::PointCloud<_PointT>::const_iterator i = c.begin(); i != c.end(); ++i)
  {
    const _PointT& p = *i;
    *output_data++ = p.r / 255.0f;
    *output_data++ = p.g / 255.0f;
    *output_data++ = p.b / 255.0f;
    *output_data++ = p.a / 255.0f;
  }
  
  return 0;
}
