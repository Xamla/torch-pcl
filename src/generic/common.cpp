#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

extern "C" {
#include <TH/TH.h>
}

PCLIMP(void*, PointCloud, new)(uint32_t width, uint32_t height) 
{
    return new pcl::PointCloud<_PointT>(width, height);
}


PCLIMP(void*, PointCloud, clone)(pcl::PointCloud<_PointT>* self)
{
    return new pcl::PointCloud<_PointT>(*self);
}


PCLIMP(void, PointCloud, delete)(pcl::PointCloud<_PointT>* self)
{
    delete self;
}


PCLIMP(uint32_t, PointCloud, width)(pcl::PointCloud<_PointT>* self)
{
    return self->width;
}


PCLIMP(uint32_t, PointCloud, height)(pcl::PointCloud<_PointT>* self)
{
    return self->height;
}


PCLIMP(uint32_t, PointCloud, isDense)(pcl::PointCloud<_PointT>* self)
{
    return self->is_dense;
}


PCLIMP(_PointT&, PointCloud, at1D)(pcl::PointCloud<_PointT>* self, int n)
{
    return self->at(n);
}


PCLIMP(_PointT&, PointCloud, at2D)(pcl::PointCloud<_PointT>* self, int column, int row)
{
    return self->at(column, row);
}


PCLIMP(void, PointCloud, clear)(pcl::PointCloud<_PointT>* self)
{
    self->clear();
}


PCLIMP(bool, PointCloud, empty)(pcl::PointCloud<_PointT>* self)
{
    return self->empty();
}


PCLIMP(bool, PointCloud, isOrganized)(pcl::PointCloud<_PointT>* self)
{
    return self->isOrganized();
}


PCLIMP(THFloatStorage*, PointCloud, points)(pcl::PointCloud<_PointT>* self)
{
    float* ptr = reinterpret_cast<float*>(&self->points[0]);
    THFloatStorage* storage = THFloatStorage_newWithData(ptr, self->points.size() * sizeof(_PointT) / sizeof(float));
    storage->flag = TH_STORAGE_REFCOUNTED;
    return storage;
}


PCLIMP(void, PointCloud, add)(pcl::PointCloud<_PointT>* self, pcl::PointCloud<_PointT>* other)
{
    *self += *other;
}


PCLIMP(THFloatStorage*, PointCloud, sensorOrigin)(pcl::PointCloud<_PointT>* self)
{
    float* ptr = &(self->sensor_origin_(0));
    THFloatStorage* storage = THFloatStorage_newWithData(ptr, 4);
    storage->flag = TH_STORAGE_REFCOUNTED;
    return storage;
}


PCLIMP(THFloatStorage*, PointCloud, sensorOrientation)(pcl::PointCloud<_PointT>* self)
{
    float* ptr = &(self->sensor_orientation_.coeffs()(0));
    THFloatStorage* storage = THFloatStorage_newWithData(ptr, 4);
    storage->flag = TH_STORAGE_REFCOUNTED;
    return storage;
}
