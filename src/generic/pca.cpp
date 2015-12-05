#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#define PCA_ptr boost::shared_ptr<pcl::PCA<_PointT> >
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr

PCLIMP(void*, PCA, new)(bool basis_only) 
{
  return new PCA_ptr(new pcl::PCA<_PointT>(basis_only));
}

PCLIMP(void*, PCA, clone)(PCA_ptr *self)
{
  return new PCA_ptr(new pcl::PCA<_PointT>(**self));
}

PCLIMP(void, PCA, delete)(PCA_ptr *self)
{
  delete self;
}

/*
PCLIMP(void, PCA, set)(PCA_ptr *self, PCA_ptr *other)
{
  **self = **other;
}
*/

PCLIMP(void, PCA, set_inputCloud)(PCA_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, PCA, get_mean)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::Vector4f& mean = (*self)->getMean();
  viewMatrix(mean, output);
}

PCLIMP(void, PCA, get_eigenVectors)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix3f& eigenVectors = (*self)->getEigenVectors();
  viewMatrix(eigenVectors, output);}

PCLIMP(void, PCA, get_eigenValues)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::Vector3f& eigenValues = (*self)->getEigenValues();
  viewMatrix(eigenValues, output);
}

PCLIMP(void, PCA, get_coefficients)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::MatrixXf& coefficients =(*self)->getCoefficients();
  viewMatrix(coefficients, output);
}

PCLIMP(void, PCA, project_cloud)(PCA_ptr *self, PointCloud_ptr *input, PointCloud_ptr *output)
{
  (*self)->project(**input, **output);
}

PCLIMP(void, PCA, reconstruct_cloud)(PCA_ptr *self, PointCloud_ptr *input, PointCloud_ptr *output)
{
  (*self)->reconstruct(**input, **output);
}

#undef PCA_ptr
#undef PointCloud_ptr
