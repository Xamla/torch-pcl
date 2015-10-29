#include <pcl/visualization/cloud_viewer.h>

PCLIMP(void, CloudViewer, showCloud)(boost::shared_ptr<pcl::visualization::CloudViewer> *self, pcl::PointCloud<_PointT>::Ptr *cloud, const char *cloud_name) 
{
  (*self)->showCloud(*cloud, cloud_name ? cloud_name : "cloud");
}
