#define TYPE_KEY _
#include "utils.h"
#include <pcl/visualization/cloud_viewer.h>

PCLIMP(boost::shared_ptr<pcl::visualization::CloudViewer>*, CloudViewer, new)(const char *window_name)
{
  return new boost::shared_ptr<pcl::visualization::CloudViewer>(new pcl::visualization::CloudViewer(window_name ? window_name : "CloudViewer"));
}

PCLIMP(void, CloudViewer, delete)(boost::shared_ptr<pcl::visualization::CloudViewer> *viewer)
{
  delete viewer;
}

PCLIMP(bool, CloudViewer, wasStopped)(boost::shared_ptr<pcl::visualization::CloudViewer> *viewer, int millis_to_wait)
{
  return (*viewer)->wasStopped(millis_to_wait);
}
