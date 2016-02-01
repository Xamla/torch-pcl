#define _PointT pcl::PointXYZ
#define _PointNormalT pcl::PointNormal
#define TYPE_KEY _XYZ_

#include "utils.h"
#include <pcl/keypoints/sift_keypoint.h>

// SIFTKeypointFieldSelector specialization to calculate SIFTKeypoint based
// on the z component of point clouds without color or intensity information.
namespace pcl
{
  template<>
  struct SIFTKeypointFieldSelector<PointXYZ>
  {
    inline float
    operator () (const PointXYZ& p) const
    {
      return p.z;
    }
  };
}

#include "generic.cpp"
#include "generic/cloudviewer.cpp"
#include "mesh_sampling.cpp"
#include "generic/openni2.cpp"
#include "generic/normal_estimation.cpp"
