#include <boost/thread/condition_variable.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "OpenNI2GrabberStream.h"

PCLIMP(void*, OpenNI2Stream, new)(const char* device_id, int max_backlog)
{
  return new OpenNI2GrabberStream<_PointT>(device_id, max_backlog);
}

PCLIMP(void, OpenNI2Stream, delete)(OpenNI2GrabberStream<_PointT> *self)
{
  delete self;
}

PCLIMP(void, OpenNI2Stream, start)(OpenNI2GrabberStream<_PointT> *self)
{
  self->start();
}

PCLIMP(void, OpenNI2Stream, stop)(OpenNI2GrabberStream<_PointT> *self)
{
  self->stop();
}

PCLIMP(void*, OpenNI2Stream, read)(OpenNI2GrabberStream<_PointT> *self, int timeout_milliseconds)
{
  pcl::PointCloud<_PointT>::Ptr cloud = self->read(timeout_milliseconds);
  if (!cloud)
    return 0;
  return new pcl::PointCloud<_PointT>::Ptr(cloud);
}
