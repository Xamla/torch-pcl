#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

extern "C" {
#include <TH/TH.h>
}

extern "C" void* pcl_PointCloud_XYZ_new(uint32_t width, uint32_t height)
{
    return new pcl::PointCloud<pcl::PointXYZ>(width, height);
}

extern "C" void pcl_PointCloud_XYZ_delete(pcl::PointCloud<pcl::PointXYZ>* self)
{
    delete self;
}

/*extern "C" pcl_PointCloud_at1D(pcl::PointCloud<pcl::PointXYZ>* self, int n)
{
    return self->at(n);
}

extern "C" pcl_PointCloud_at2D(pcl::PointCloud<pcl::PointXYZ>* self, int column, int row)
{
    return self->at(column, row);
}*/

// set_at1D
// set_at2D


extern "C" void pcl_PointCloud_XYZ_clear(pcl::PointCloud<pcl::PointXYZ>* self)
{
    self->clear();
}

extern "C" bool pcl_PointCloud_XYZ_empty(pcl::PointCloud<pcl::PointXYZ>* self)
{
    return self->empty();
}

extern "C" bool pcl_PointCloud_XYZ_isOrganized(pcl::PointCloud<pcl::PointXYZ>* self)
{
    return self->isOrganized();
}

extern "C" THFloatStorage* pcl_PointCloud_XYZ_storage(pcl::PointCloud<pcl::PointXYZ>* self)
{
    float* ptr = reinterpret_cast<float*>(&self->points[0]);
    THFloatStorage* storage = THFloatStorage_newWithData(ptr, self->points.size() * sizeof(pcl::PointXYZ));
    storage->refcount = 1;
    storage->flag = TH_STORAGE_REFCOUNTED;
    return storage;
}

/*extern "C" void pcl_PointCloud_XYZ_tensormap(pcl::PointCloud<pcl::PointXYZ>* self, void* destination, int dim, int stride, int offset)
{
    THTensor* d = static_cast<THTensor*>(destination);
    
    //reinterpret_cast<float*>(&self->points[0]) + offset, 
    //self->points.size(), dim, 
    //stride
}

extern "C" void pcl_PointCloud_XYZ_totensor(pcl::PointCloud<pcl::PointXYZ>* self, void* destination)
{
    pcl_PointCloud_XYZ_tensormap(self, destination, sizeof(pcl::PointXYZ) / sizeof(float), sizeof(pcl::PointXYZ) / sizeof(float), 0));
}
*/
//extern "C" pcl_PointCloud_XYZ_op_add(pcl::PointCloud<pcl::PointXYZ>* self, pcl::PointCloud<pcl::PointXYZ>* other)
//{
//}
