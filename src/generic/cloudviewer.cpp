#include <pcl/visualization/cloud_viewer.h>

PCLIMP(void, CloudViewer, showCloud)(pcl::visualization::CloudViewer *self, pcl::PointCloud<_PointT>::Ptr *cloud, const char *cloudname) 
{
  self->showCloud(*cloud, cloudname ? cloudname : "cloud");
}
