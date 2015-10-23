#define _PointT pcl::PointXYZI
#define TYPE_KEY _XYZI_
#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)

#include "generic/common.cpp"