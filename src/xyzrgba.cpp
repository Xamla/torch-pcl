#define _PointT pcl::PointXYZRGBA
#define TYPE_KEY _XYZRGBA_
#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)

#include "generic/common.cpp"
#include "generic/io.cpp"
