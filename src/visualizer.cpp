#define TYPE_KEY _
#include "utils.h"
#include <pcl/visualization/pcl_visualizer.h>

#define PCLVisualizer_ptr pcl::visualization::PCLVisualizer::Ptr

PCLIMP(PCLVisualizer_ptr *, PCLVisualizer, new)(const char *name, bool create_interactor)
{
  return new PCLVisualizer_ptr(new pcl::visualization::PCLVisualizer(name, create_interactor));
}

PCLIMP(void, PCLVisualizer, delete)(PCLVisualizer_ptr *self)
{
  delete self;
}

PCLIMP(void, PCLVisualizer, setFullScreen)(PCLVisualizer_ptr *self, bool mode)
{
  (*self)->setFullScreen(mode);
}

PCLIMP(void, PCLVisualizer, setWindowName)(PCLVisualizer_ptr *self, const char *name)
{
  (*self)->setWindowName(name);
}

PCLIMP(void, PCLVisualizer, setWindowBorders)(PCLVisualizer_ptr *self, bool mode)
{
  (*self)->setWindowBorders(mode);
}

PCLIMP(void, PCLVisualizer, spin)(PCLVisualizer_ptr *self)
{
  (*self)->spin();
}

PCLIMP(void, PCLVisualizer, spinOnce)(PCLVisualizer_ptr *self, int time = 1, bool force_redraw = false)
{
  (*self)->spinOnce(time, force_redraw);
}

PCLIMP(void, PCLVisualizer, addCoordinateSystem)(PCLVisualizer_ptr *self, double scale = 1.0, const char *id = "reference", int viewport = 0)
{
  (*self)->addCoordinateSystem(scale, id, viewport);
}
  
PCLIMP(int, PCLVisualizer, createViewPort)(PCLVisualizer_ptr *self, double xmin, double ymin, double xmax, double ymax)
{
  int viewport = 0;
  (*self)->createViewPort(xmin, ymin, xmax, ymax, viewport);
  return viewport;
}

PCLIMP(void, PCLVisualizer, createViewPortCamera)(PCLVisualizer_ptr *self, int viewport)
{
  (*self)->createViewPortCamera(viewport);
}

PCLIMP(void, PCLVisualizer, setBackgroundColor)(PCLVisualizer_ptr *self, double r, double g, double b, int viewport = 0)
{
  (*self)->setBackgroundColor(r, g, b, viewport);
}

PCLIMP(bool, PCLVisualizer, removeAllPointClouds)(PCLVisualizer_ptr *self, int viewport = 0)
{
  return (*self)->removeAllPointClouds(viewport);
}

PCLIMP(bool, PCLVisualizer, removeAllShapes)(PCLVisualizer_ptr *self, int viewport = 0)
{
  return (*self)->removeAllShapes(viewport);
}

PCLIMP(bool, PCLVisualizer, removeAllCoordinateSystems)(PCLVisualizer_ptr *self, int viewport = 0)
{
  return (*self)->removeAllCoordinateSystems(viewport);
}

PCLIMP(bool, PCLVisualizer, addText1)(PCLVisualizer_ptr *self, const char *text, int xpos, int ypos, const char *id = "", int viewport = 0)
{
  return (*self)->addText(text, xpos, ypos, id, viewport);
}
                 
PCLIMP(bool, PCLVisualizer, addText2)(PCLVisualizer_ptr *self, const char *text, int xpos, int ypos, double r, double g, double b, const char *id = "", int viewport = 0)
{
  return (*self)->addText(text, xpos, ypos, r, g, b, id, viewport);
}
                 
PCLIMP(bool, PCLVisualizer, addText3)(PCLVisualizer_ptr *self, const char *text, int xpos, int ypos, int fontsize, double r, double g, double b, const char *id = "", int viewport = 0)
{
  return (*self)->addText(text, xpos, ypos, fontsize, r, g, b, id, viewport);
}

PCLIMP(void, PCLVisualizer, initCameraParameters)(PCLVisualizer_ptr *self)
{
  (*self)->initCameraParameters();
}

PCLIMP(bool, PCLVisualizer, setPointCloudRenderingProperties1)(PCLVisualizer_ptr *self, int property, double value, const char *id = "cloud", int viewport = 0)
{
  return (*self)->setPointCloudRenderingProperties(property, value, id, viewport);
}

PCLIMP(bool, PCLVisualizer, setPointCloudRenderingProperties2)(PCLVisualizer_ptr *self, int property, double val1, double val2, double val3, const char *id = "cloud", int viewport = 0)
{
  return (*self)->setPointCloudRenderingProperties(property, val1, val2, val3, id, viewport);
}
                                          
PCLIMP(bool, PCLVisualizer, wasStopped)(PCLVisualizer_ptr *self)
{
 return (*self)->wasStopped();
}

PCLIMP(void, PCLVisualizer, resetStoppedFlag)(PCLVisualizer_ptr *self)
{
  (*self)->resetStoppedFlag();
}

PCLIMP(void, PCLVisualizer, close)(PCLVisualizer_ptr *self)
{
  (*self)->close();
}

#undef PCLVisualizer_ptr
