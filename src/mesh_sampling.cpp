#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkConeSource.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkPlaneSource.h>
#include <vtkDiskSource.h>
#include <vtkTriangleFilter.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

inline double uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void randomPointTriangle(
  float a1, float a2, float a3, 
  float b1, float b2, float b3, 
  float c1, float c2, float c3,
  Eigen::Vector4f& p)
{
  float r1 = static_cast<float>(uniform_deviate(rand()));
  float r2 = static_cast<float>(uniform_deviate(rand()));
  float r1sqr = sqrtf (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void randPSurface(
  vtkPolyData *polydata, 
  std::vector<double> *cumulativeAreas, 
  double totalArea, 
  Eigen::Vector4f& p)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]), 
                       float (B[0]), float (B[1]), float (B[2]), 
                       float (C[0]), float (C[1]), float (C[2]), p);
}

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    randPSurface (polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

void sampleMesh(vtkPolyData* polydata, pcl::PointCloud<pcl::PointXYZ>::Ptr output, int samples = 100000, float resolution = 0.1f)
{
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInput(polydata);
  triangleFilter->Update();
  polydata = triangleFilter->GetOutput();
  
  // sample into new point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  uniform_sampling(polydata, samples, *cloud);
  
  // create uniform voxel grid
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(cloud);
  grid.setLeafSize(resolution, resolution, resolution);

  grid.filter(*output);
}

PCLIMP(void, Primitive, createSphere)(pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  double radius,    // Set radius of sphere.
  double thetaRes,  // Set the number of points in the longitude direction.
  double phiRes,    // Set the number of points in the latitude direction.
  int samples, float resolution)
{
  // Create a sphere
  vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
  sphere->SetCenter(0.0, 0.0, 0.0);
  sphere->SetRadius(radius);
  sphere->SetThetaResolution(thetaRes);
  sphere->SetPhiResolution(phiRes);
  sphere->Update();
  sampleMesh(sphere->GetOutput(), *output, samples, resolution);
}

PCLIMP(void, Primitive, createCube)(pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  double x,   // Set the length of the cube in the x-direction.
  double y,   // Set the length of the cube in the y-direction.
  double z,   // Set the length of the cube in the z-direction.
  int samples, float resolution)
{
  vtkSmartPointer<vtkCubeSource> cube= vtkSmartPointer<vtkCubeSource>::New();
  cube->SetXLength(x);
  cube->SetYLength(y);
  cube->SetZLength(z);
  cube->Update();
  sampleMesh(cube->GetOutput(), *output, samples, resolution);
}

PCLIMP(void, Primitive, createCylinder)(pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  double height,  // Set the height of the cylinder
  double radius,  // Set the radius of the cylinder
  int facets,     // Set the number of facets used to define cylinder.
  int samples, float resolution)
{
  vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();  
  cylinder->SetHeight(height);
  cylinder->SetRadius(radius);
  cylinder->SetResolution(facets);
  cylinder->Update();
  sampleMesh(cylinder->GetOutput(), *output, samples, resolution);
}

PCLIMP(void, Primitive, createCone)(pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  double height,    // Set the height of the cone.
  double radius,    // Set the base radius of the cone.
  int facets,       // Set the number of facets used to represent the cone.
  int samples, float resolution)
{
  vtkSmartPointer<vtkConeSource> cone = vtkSmartPointer<vtkConeSource>::New();
  cone->SetHeight(height);
  cone->SetRadius(radius);
  cone->SetResolution(facets);
  cone->Update();
  sampleMesh(cone->GetOutput(), *output, samples, resolution);
}
  
PCLIMP(void, Primitive, createPlatonicSolid)(pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  int solidType,  // Specify the type of PlatonicSolid solid to create.
  int samples, float resolution)
{
  vtkSmartPointer<vtkPlatonicSolidSource> platonicSolid = vtkSmartPointer<vtkPlatonicSolidSource>::New();
  platonicSolid->SetSolidType(solidType);
  platonicSolid->Update();
  sampleMesh(platonicSolid->GetOutput(), *output, samples, resolution);
}

PCLIMP(void, Primitive, createPlane)(pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  double x1, double y1, double z1,  // Specify a point defining the first axis of the plane.
  double x2, double y2, double z2,  // Specify a point defining the second axis of the plane.
  int samples, float resolution)
{
  vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
  plane->SetOrigin(0, 0, 0);
  plane->SetPoint1(x1, y1, z1);
  plane->SetPoint2(x2, y2, z2);
  plane->Update();
  sampleMesh(plane->GetOutput(), *output, samples, resolution);
}

PCLIMP(void, Primitive, createDisk)(
  pcl::PointCloud<pcl::PointXYZ>::Ptr *output, 
  double innerRadius,             // Specify inner radius of hole in disc.
  double outerRadius,             // Specify outer radius of disc.
  int circumferentialResolution,  // Set the number of points in circumferential direction.
  int samples, float resolution)
{
  vtkSmartPointer<vtkDiskSource> disk = vtkSmartPointer<vtkDiskSource>::New();
  disk->SetInnerRadius(innerRadius);
  disk->SetOuterRadius(outerRadius);
  disk->SetCircumferentialResolution(circumferentialResolution);
  disk->Update();
  sampleMesh(disk->GetOutput(), *output, samples, resolution);
}
