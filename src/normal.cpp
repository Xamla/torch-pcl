#define _PointT pcl::Normal
#define TYPE_KEY _Normal_
#define _PointT_WITHOUT_XYZ
#define _PointT_HAS_NORMALS
#include "utils.h"
#include "generic/PointCloud.cpp"

PCLIMP(void, PointCloud, copyNormal)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::Normal>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}

PCLIMP(void, PointCloud, copyXYZNormal)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointNormal>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}

PCLIMP(void, PointCloud, copyXYZINormal)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointXYZINormal>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}

PCLIMP(void, PointCloud, copyXYZRGBNormal)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}
