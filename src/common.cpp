#include <pcl/point_types.h>

extern "C" {
#include <TH/TH.h>
}

#define _PointT pcl:PointXYZ
#define TYPE_KEY XYZ_
#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)


PCLIMP(void*, PointCloud, new)(uint32_t width, uint32_t height) 
{
    return new pcl::PointCloud<_PointT>(width, height);
}

PCLIMP(void, PointCloud, delete)(pcl::PointCloud<_PointT>* self)
{
    delete self;
}

PCLIMP(_PointT&, PointCloud, at1D)(pcl::PointCloud<_PointT>* self, int n)
{
    return self->at(n);
}

/*extern "C" pcl_PointCloud_at2D(pcl::PointCloud<pcl::PointXYZ>* self, int column, int row)
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
    THFloatStorage* storage = THFloatStorage_newWithData(ptr, self->points.size() * sizeof(pcl::PointXYZ) / sizeof(float));
    storage->flag = TH_STORAGE_REFCOUNTED;
    return storage;
}

//extern "C" pcl_PointCloud_XYZ_op_add(pcl::PointCloud<pcl::PointXYZ>* self, pcl::PointCloud<pcl::PointXYZ>* other)
//{
//}
