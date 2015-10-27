#define TYPE_KEY _ALL_
#include "utils.h"
#include <pcl/visualization/cloud_viewer.h>

PCLIMP(pcl::visualization::CloudViewer*, CloudViewer, new)(const char* window_name)
{
  return new pcl::visualization::CloudViewer(window_name);
}

PCLIMP(void, CloudViewer, delete)(pcl::visualization::CloudViewer* viewer)
{
  delete viewer;
}
