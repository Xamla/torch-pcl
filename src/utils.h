#ifndef utils_h
#define utils_h

extern "C" {
#include <TH/TH.h>
}

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/exceptions.h>
#include <pcl/pcl_base.h>
#include "xyzrgbanormal.h"

#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)
#define PCLCALL(class_name, name) TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)
#define Indices_ptr pcl::IndicesPtr
#define Normals_ptr pcl::PointCloud<pcl::Normal>::Ptr

class TorchPclException : public pcl::PCLException
{
  public:
    TorchPclException(const std::string &error_description, const char *file_name = NULL, const char *function_name = NULL, unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number)
    {
    }
};

inline Eigen::Vector4f Tensor2Vec4f(THFloatTensor *tensor)
{
  if (THFloatTensor_nElement(tensor) < 4)
    PCL_THROW_EXCEPTION(TorchPclException, "A tensor with at least 4 elements was expected.");
    
  THFloatTensor *tensor_ = THFloatTensor_newContiguous(tensor);
  float* data = THFloatTensor_data(tensor_);
  Eigen::Vector4f v(data[0], data[1], data[2], data[3]);
  THFloatTensor_free(tensor_);
  return v;
}

inline Eigen::Vector3f Tensor2Vec3f(THFloatTensor *tensor)
{
  if (THFloatTensor_nElement(tensor) < 4)
    PCL_THROW_EXCEPTION(TorchPclException, "A Tensor with at least 3 elements was expected.");
    
  THFloatTensor *tensor_ = THFloatTensor_newContiguous(tensor);
  float* data = THFloatTensor_data(tensor_);
  Eigen::Vector3f v(data[0], data[1], data[2]);
  THFloatTensor_free(tensor_);
  return v;
}

template<int rows, int cols>
inline Eigen::Matrix<float, rows, cols> Tensor2Mat(THFloatTensor *tensor)
{
  THArgCheck(tensor != NULL && tensor->nDimension == 2 && tensor->size[0] == rows && tensor->size[1] == cols, 1, "invalid tensor");
  tensor = THFloatTensor_newContiguous(tensor);
  Eigen::Matrix<float, rows, cols> output(Eigen::Map<Eigen::Matrix<float, rows, cols, Eigen::RowMajor> >(THFloatTensor_data(tensor)));
  THFloatTensor_free(tensor);
  return output;
}

template<int rows, int cols, int options> void viewMatrix(Eigen::Matrix<float, rows, cols, options> &m, THFloatTensor *output)
{
  // create new storage that views into the matrix
  THFloatStorage* storage = NULL;
  if ((Eigen::Matrix<float, rows, cols, options>::Options & Eigen::RowMajor) == Eigen::RowMajor)
    storage = THFloatStorage_newWithData(m.data(), (m.rows() * m.rowStride()));
  else
    storage = THFloatStorage_newWithData(m.data(), (m.cols() * m.colStride()));
    
  storage->flag = TH_STORAGE_REFCOUNTED;
  THFloatTensor_setStorage2d(output, storage, 0, rows, m.rowStride(), cols, m.colStride());
  THFloatStorage_free(storage);   // tensor took ownership
}

template<int rows, int cols> void copyMatrix(const Eigen::Matrix<float, rows, cols> &m, THFloatTensor *output)
{
  THFloatTensor_resize2d(output, m.rows(), m.cols());
  THFloatTensor* output_ = THFloatTensor_newContiguous(output);
  // there are strange static-asserts in Eigen to disallow specifying RowMajor for vectors...
  Eigen::Map<Eigen::Matrix<float, rows, cols, (rows == 1 || cols == 1) ? Eigen::ColMajor : Eigen::RowMajor> >(THFloatTensor_data(output)) = m;
  THFloatTensor_freeCopyTo(output_, output);
}

inline void vector2Tensor(const std::vector<int> &v, THIntTensor *output)
{
  THIntTensor_resize1d(output, v.size());
  THIntTensor* output_ = THIntTensor_newContiguous(output);
  std::copy(v.begin(), v.end(), THIntTensor_data(output_));
  THIntTensor_freeCopyTo(output_, output);
}

inline void vector2Tensor(const std::vector<float> &v, THFloatTensor *output)
{
  THFloatTensor_resize1d(output, v.size());
  THFloatTensor* output_ = THFloatTensor_newContiguous(output);
  std::copy(v.begin(), v.end(), THFloatTensor_data(output_));
  THFloatTensor_freeCopyTo(output_, output);
}

#endif
