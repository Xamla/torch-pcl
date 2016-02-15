#include <boost/thread/condition_variable.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "../openni2.h"
#include "OpenNI2GrabberStream.h"

PCLIMP(void*, OpenNI2Stream, new)(const char* device_id, int max_backlog, bool grab_RGB = false, bool grab_Depth = false, bool grab_IR = false)
{
  try
  {
    return new OpenNI2GrabberStream<_PointT>(device_id, max_backlog, grab_RGB, grab_Depth, grab_IR);
  }
  catch (const std::exception& ex)
  {
    // ## temporary debug output, remove as soon as C++ exception translation method is implemented
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
  try
  {
    self->start();
  }
  catch (const std::exception& ex)
  {
    // ## temporary debug output, remove as soon as C++ exception translation method is implemented
    std::cout << ex.what() << std::endl;
    throw;
  }
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

PCLIMP(void, OpenNI2Stream, readRGBImage)(OpenNI2GrabberStream<_PointT> *self, int timeout_milliseconds, THByteTensor* output)
{
  pcl::io::Image::ConstPtr ptr = self->readRGBImage(timeout_milliseconds);
  if (ptr)
  {
    unsigned int height = ptr->getHeight(), width = ptr->getWidth();
    THByteTensor_resize3d(output, height, width, 3);
    THByteTensor* output_ = THByteTensor_newContiguous(output);
    ptr->fillRGB(width, height, THByteTensor_data(output_));
    THByteTensor_freeCopyTo(output_, output);
  }
}

PCLIMP(void, OpenNI2Stream, readDepthImage)(OpenNI2GrabberStream<_PointT> *self, int timeout_milliseconds, THShortTensor* output)
{
  pcl::io::DepthImage::ConstPtr ptr = self->readDepthImage(timeout_milliseconds);
  if (ptr)
  {
    unsigned int height = ptr->getHeight(), width = ptr->getWidth();
    THShortTensor_resize3d(output, height, width, 1);
    THShortTensor* output_ = THShortTensor_newContiguous(output);
    ptr->fillDepthImageRaw(width, height, reinterpret_cast<unsigned short*>(THShortTensor_data(output_)));
    THShortTensor_freeCopyTo(output_, output);
  }
}

PCLIMP(void, OpenNI2Stream, readIRImage)(OpenNI2GrabberStream<_PointT> *self, int timeout_milliseconds, THShortTensor* output)
{
  pcl::io::IRImage::ConstPtr ptr = self->readIRImage(timeout_milliseconds);
  if (ptr)
  {
    unsigned int height = ptr->getHeight(), width = ptr->getWidth();
    THShortTensor_resize3d(output, height, width, 1);
    THShortTensor* output_ = THShortTensor_newContiguous(output);
    ptr->fillRaw(width, height, reinterpret_cast<unsigned short*>(THShortTensor_data(output_)));
    THShortTensor_freeCopyTo(output_, output);
  }
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
