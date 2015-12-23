#ifndef xyzrgbanormal_h
#define xyzrgbanormal_h

// define the missing PointXYZRGBANormal point type
#include <pcl/point_types.h>

struct PointXYZRGBANormal;

struct EIGEN_ALIGN16 _PointXYZRGBANormal
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      PCL_ADD_UNION_RGB;
      float curvature;
    };
    float data_c[4];
  };
  PCL_ADD_EIGEN_MAPS_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBANormal& p);

struct PointXYZRGBANormal : public _PointXYZRGBANormal
{
  inline PointXYZRGBANormal (const _PointXYZRGBANormal &p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
    curvature = p.curvature;
    rgba = p.rgba;
  }

  inline PointXYZRGBANormal ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    rgba = 0;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    curvature = 0;
  }

  friend std::ostream& operator << (std::ostream& os, const PointXYZRGBANormal& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT (_PointXYZRGBANormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, rgba, rgba)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(PointXYZRGBANormal, _PointXYZRGBANormal)

#endif // xyzrgbanormal_h
