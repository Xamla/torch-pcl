#define _PointT pcl::PointXYZRGBNormal
#define TYPE_KEY _XYZRGBNormal_
#define _PointT_HAS_NORMALS

#include "utils.h"
#include <pcl/keypoints/sift_keypoint.h>

namespace pcl
{
  template<>
  struct SIFTKeypointFieldSelector<PointXYZRGBNormal>
  {
    inline float
    operator () (const PointXYZRGBNormal& p) const
    {
      return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) / 1000.0f);
    }
  };
}

#include "generic.cpp"
#include "generic/openni2.cpp"
