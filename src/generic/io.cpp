#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

extern "C" {
#include <TH/TH.h>
}

PCLIMP(int, PointCloud, loadPCDFile)(pcl::PointCloud<_PointT> *cloud, const char *fn)
{
    return pcl::io::loadPCDFile<_PointT> (fn, *cloud);
}

PCLIMP(int, PointCloud, savePCDFile)(pcl::PointCloud<_PointT> *cloud, const char *fn, bool binary) 
{
    return pcl::io::savePCDFile(fn, *cloud, binary);
}

PCLIMP(int, PointCloud, loadPLYFile)(pcl::PointCloud<_PointT> *cloud, const char *fn)
{
    return pcl::io::loadPLYFile(fn, *cloud);
}

PCLIMP(int, PointCloud, savePLYFile)(pcl::PointCloud<_PointT> *cloud, const char *fn, bool binary)
{
    return pcl::io::savePLYFile(fn, *cloud, binary);
}

PCLIMP(int, PointCloud, loadOBJFile)(pcl::PointCloud<_PointT> *cloud, const char *fn)
{
    return pcl::io::loadOBJFile(fn, *cloud);
}

PCLIMP(void, PointCloud, savePNGFile)(pcl::PointCloud<_PointT> *cloud, const char *fn, const char* field_name)
{
    pcl::io::savePNGFile(fn, *cloud, field_name);
}
