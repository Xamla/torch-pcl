#define _PointT pcl::PointXYZRGBA
#define TYPE_KEY _XYZRGBA_
#define _PointNormalT pcl::PointXYZRGBNormal
#define _PointT_HAS_COLOR

#include "utils.h"
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace pcl
{
  template<>
  void concatenateFields (
    const pcl::PointCloud<pcl::PointXYZRGBA> &cloud1_in, 
    const pcl::PointCloud<pcl::Normal> &cloud2_in, 
    pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out)
  {
    if (cloud1_in.points.size () != cloud2_in.points.size ())
    {
      PCL_ERROR ("[pcl::concatenateFields] The number of points in the two input datasets differs!\n");
      return;
    }

    // Resize the output dataset
    cloud_out.points.resize (cloud1_in.points.size ());
    cloud_out.header   = cloud1_in.header;
    cloud_out.width    = cloud1_in.width;
    cloud_out.height   = cloud1_in.height;
    if (!cloud1_in.is_dense || !cloud2_in.is_dense)
      cloud_out.is_dense = false;
    else
      cloud_out.is_dense = true;

    // Iterate over each point
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      const pcl::PointXYZRGBA &x1 = cloud1_in[i];
      const pcl::Normal       &x2 = cloud2_in[i];
      pcl::PointXYZRGBNormal  &y  = cloud_out[i];
      memcpy(y.data, x1.data, sizeof(x1.data));         // coordinates
      memcpy(y.data_n, x2.data_n, sizeof(x2.data_n));   // normal
      y.rgba = x1.rgba;                                 // color
      y.curvature = x2.curvature;                       // curvature
    }
  }
}

#include "generic.cpp"
#include "generic/cloudviewer.cpp"
#include "generic/openni2.cpp"
#include "generic/normal_estimation.cpp"
#include "generic/readRGBA.cpp"
