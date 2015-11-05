#ifndef utils_h
#define utils_h

extern "C" {
#include <TH/TH.h>
}

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)

inline Eigen::Vector4f Tensor2Vec4f(THFloatTensor *tensor)
{
  return Eigen::Vector4f(
    THFloatTensor_get1d(tensor, 1),
    THFloatTensor_get1d(tensor, 2),
    THFloatTensor_get1d(tensor, 3),
    THFloatTensor_get1d(tensor, 4)
  );
}

inline Eigen::Vector3f Tensor2Vec3f(THFloatTensor *tensor)
{
  return Eigen::Vector3f(
    THFloatTensor_get1d(tensor, 1),
    THFloatTensor_get1d(tensor, 2),
    THFloatTensor_get1d(tensor, 3)
  );
}

#endif