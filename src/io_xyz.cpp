#define _PointT pcl::PointXYZ
#define TYPE_KEY XYZ_
#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)

#include "generic/io.cpp"
