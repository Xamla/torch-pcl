#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

PCLIMP(int, PointCloud, loadPCDFile)(pcl::PointCloud<_PointT>::Ptr *self, const char *fn)
{
    return pcl::io::loadPCDFile<_PointT>(fn, **self);
}

PCLIMP(int, PointCloud, savePCDFile)(pcl::PointCloud<_PointT>::Ptr *self, const char *fn, bool binary) 
{
    return pcl::io::savePCDFile(fn, **self, binary);
}

PCLIMP(int, PointCloud, loadPLYFile)(pcl::PointCloud<_PointT>::Ptr *self, const char *fn)
{
    return pcl::io::loadPLYFile(fn, **self);
}

PCLIMP(int, PointCloud, savePLYFile)(pcl::PointCloud<_PointT>::Ptr *self, const char *fn, bool binary)
{
    return pcl::io::savePLYFile(fn, **self, binary);
}

PCLIMP(int, PointCloud, loadOBJFile)(pcl::PointCloud<_PointT>::Ptr *self, const char *fn)
{
    return pcl::io::loadOBJFile(fn, **self);
}

PCLIMP(void, PointCloud, savePNGFile)(pcl::PointCloud<_PointT>::Ptr *self, const char *fn, const char* field_name)
{
    pcl::io::savePNGFile(fn, **self, field_name);
}

PCLIMP(void, PointCloud, copyXYZ)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}

PCLIMP(void, PointCloud, copyXYZI)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointXYZI>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}

PCLIMP(void, PointCloud, copyXYZRGB)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr *cloud_out)
{
  if (indices)
    pcl::copyPointCloud(**self, **indices, **cloud_out);
  else
    pcl::copyPointCloud(**self, **cloud_out);
}

PCLIMP(void, PointCloud, copyXYZRGBA)(pcl::PointCloud<_PointT>::Ptr *self, Indices_ptr *indices, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr *cloud_out)
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
