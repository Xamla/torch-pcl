#include <boost/thread/condition_variable.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "../openni2.h"
#include "OpenNI2GrabberStream.h"

PCLIMP(void*, OpenNI2Stream, new)(const char* device_id, int max_backlog)
{
  try
  {
    return new OpenNI2GrabberStream<_PointT>(device_id, max_backlog);
  }
  catch (const std::exception& ex)
  {
    // ## temporary debug output
    std::cout << ex.what() << std::endl;
    throw;
  }
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

PCLIMP(void, OpenNI2Stream, getRGBCameraIntrinsics)(OpenNI2GrabberStream<_PointT> *self, OpenNI2CameraParameters& p)
{
  self->getGrabber().getRGBCameraIntrinsics(p.focal_length_x, p.focal_length_y, p.principal_point_x, p.principal_point_y);
}

PCLIMP(void, OpenNI2Stream, setRGBCameraIntrinsics)(OpenNI2GrabberStream<_PointT> *self, const OpenNI2CameraParameters& p)
{
  self->getGrabber().setRGBCameraIntrinsics(p.focal_length_x, p.focal_length_y, p.principal_point_x, p.principal_point_y);
}

PCLIMP(void, OpenNI2Stream, getDepthCameraIntrinsics)(OpenNI2GrabberStream<_PointT> *self, OpenNI2CameraParameters& p)
{
  self->getGrabber().getDepthCameraIntrinsics(p.focal_length_x, p.focal_length_y, p.principal_point_x, p.principal_point_y);
}

PCLIMP(void, OpenNI2Stream, setDepthCameraIntrinsics)(OpenNI2GrabberStream<_PointT> *self, const OpenNI2CameraParameters& p)
{
  self->getGrabber().setDepthCameraIntrinsics(p.focal_length_x, p.focal_length_y, p.principal_point_x, p.principal_point_y);
}

PCLIMP(const char*, OpenNI2Stream, getName)(OpenNI2GrabberStream<_PointT> *self)
{
  return self->getGrabber().getName().c_str();
}

PCLIMP(float, OpenNI2Stream, getFramesPerSecond)(OpenNI2GrabberStream<_PointT> *self)
{
  return self->getGrabber().getFramesPerSecond();
}
