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

PCLIMP(void, PCLVisualizer, setShowFPS)(PCLVisualizer_ptr *self, bool show_fps)
{
  (*self)->setShowFPS(show_fps);
}

PCLIMP(void, PCLVisualizer, updateCamera)(PCLVisualizer_ptr *self)
{
  (*self)->updateCamera();
}

PCLIMP(void, PCLVisualizer, resetCamera)(PCLVisualizer_ptr *self)
{
  (*self)->resetCamera();
}

PCLIMP(void, PCLVisualizer, resetCameraViewpoint)(PCLVisualizer_ptr *self, const char *id = "cloud")
{
  (*self)->resetCameraViewpoint(id);
}

PCLIMP(void, PCLVisualizer, setCameraPosition)(PCLVisualizer_ptr *self,  
  double pos_x, double pos_y, double pos_z,
  double view_x, double view_y, double view_z,
  double up_x, double up_y, double up_z, int viewport = 0)
{
  (*self)->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z, viewport);
}

PCLIMP(void, PCLVisualizer, setCameraClipDistances)(PCLVisualizer_ptr *self, double near, double far, int viewport = 0)
{
  (*self)->setCameraClipDistances(near, far, viewport);
}

PCLIMP(void, PCLVisualizer, setCameraFieldOfView)(PCLVisualizer_ptr *self, double fovy, int viewport = 0)
{
  (*self)->setCameraFieldOfView(fovy, viewport);
}

PCLIMP(void, PCLVisualizer, setCameraParameters_Tensor)(PCLVisualizer_ptr *self, THFloatTensor *intrinsics, THFloatTensor *extrinsics, int viewport = 0)
{
  Eigen::Matrix3f intrinsics_ = Tensor2Mat<3,3>(intrinsics);
  Eigen::Matrix4f extrinsics_ = Tensor2Mat<4,4>(extrinsics);
  (*self)->setCameraParameters(intrinsics_, extrinsics_, viewport);
}

PCLIMP(void, PCLVisualizer, saveScreenshot)(PCLVisualizer_ptr *self, const char *fn)
{
  (*self)->saveScreenshot(fn);
}

PCLIMP(void, PCLVisualizer, saveCameraParameters)(PCLVisualizer_ptr *self, const char *fn)
{
  (*self)->saveCameraParameters(fn);
}

PCLIMP(bool, PCLVisualizer, loadCameraParameters)(PCLVisualizer_ptr *self, const char *fn)
{
  return (*self)->loadCameraParameters(fn);
}

PCLIMP(void, PCLVisualizer, getViewerPose)(PCLVisualizer_ptr *self, int viewport, THFloatTensor *result)
{
  const Eigen::Affine3f &pose = (*self)->getViewerPose();
  copyMatrix(pose.matrix(), result);}

PCLIMP(bool, PCLVisualizer, addPlane_Coefficients)(PCLVisualizer_ptr *self, THFloatTensor *coefficients, 
  double x, double y, double z, const char *id = "plane", int viewport = 0)
{
  pcl::ModelCoefficients c;
  Tensor2vector(coefficients, c.values);
  return (*self)->addPlane(c, x, y, z, id, viewport);
}

PCLIMP(bool, PCLVisualizer, addLine_Coefficients)(PCLVisualizer_ptr *self, THFloatTensor *coefficients, 
  const char *id = "line", int viewport = 0)
{
  pcl::ModelCoefficients c;
  Tensor2vector(coefficients, c.values);
  return (*self)->addLine(c, id, viewport);
}

PCLIMP(bool, PCLVisualizer, addSphere_Coefficients)(PCLVisualizer_ptr *self, THFloatTensor *coefficients,
  const char *id = "sphere", int viewport = 0)
{
  pcl::ModelCoefficients c;
  Tensor2vector(coefficients, c.values);
  return (*self)->addSphere(c, id, viewport);
}

PCLIMP(bool, PCLVisualizer, addCube_Coefficients)(PCLVisualizer_ptr *self, THFloatTensor *coefficients,
  const char *id = "cube", int viewport = 0)
{
  pcl::ModelCoefficients c;
  Tensor2vector(coefficients, c.values);
  return (*self)->addCube(c, id, viewport);
}

PCLIMP(bool, PCLVisualizer, addCylinder_Coefficients)(PCLVisualizer_ptr *self, THFloatTensor *coefficients,
  const char *id = "cylinder", int viewport = 0)
{
  pcl::ModelCoefficients c;
  Tensor2vector(coefficients, c.values);
  return (*self)->addCylinder(c, id, viewport);
}

PCLIMP(bool, PCLVisualizer, addCube)(PCLVisualizer_ptr *self, 
  float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
  double r = 1.0, double g = 1.0, double b = 1.0, const char *id = "cube", int viewport = 0)
{
  return (*self)->addCube(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, id, viewport);
}

PCLIMP(void, PCLVisualizer, setRepresentationToSurfaceForAllActors)(PCLVisualizer_ptr *self)
{
  (*self)->setRepresentationToSurfaceForAllActors();
}

PCLIMP(void, PCLVisualizer, setRepresentationToPointsForAllActors)(PCLVisualizer_ptr *self)
{
  (*self)->setRepresentationToPointsForAllActors();
}

PCLIMP(void, PCLVisualizer, setRepresentationToWireframeForAllActors)(PCLVisualizer_ptr *self)
{
  (*self)->setRepresentationToWireframeForAllActors();
}

typedef void (*MouseEventCallback)(int type, int button, int x, int y, bool alt, bool ctrl, bool shift, bool selection_mode);

static void mouse_event_translator(const pcl::visualization::MouseEvent& event, void  *cookie)
{
  MouseEventCallback target = reinterpret_cast<MouseEventCallback>(cookie);
  unsigned int mod = event.getKeyboardModifiers();
  target((int)event.getType(), (int)event.getButton(), event.getX(), event.getY(), mod & 1, mod & 2, mod & 4, event.getSelectionMode());
}

PCLIMP(boost::signals2::connection *, PCLVisualizer, registerMouseCallback)(PCLVisualizer_ptr *self, MouseEventCallback callback)
{
  boost::signals2::connection connection = (*self)->registerMouseCallback(mouse_event_translator, reinterpret_cast<void*>(callback));
  return new boost::signals2::connection(connection);
}

PCLIMP(void, PCLVisualizer, unregisterCallback)(boost::signals2::connection *connection)
{
  connection->disconnect(); 
}

PCLIMP(void, PCLVisualizer, deleteCallback)(boost::signals2::connection *connection)
{
  delete connection;
}

#undef PCLVisualizer_ptr
