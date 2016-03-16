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

PCLIMP(void, PCA, setInputCloud)(PCA_ptr *self, PointCloud_ptr *cloud)
{
  (*self)->setInputCloud(*cloud);
}

PCLIMP(void, PCA, setIndices)(PCA_ptr *self, Indices_ptr *indices)
{
  (*self)->setIndices(*indices);
}

PCLIMP(void, PCA, getMean)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::Vector4f& mean = (*self)->getMean();
  copyMatrix(mean, output);
}

PCLIMP(void, PCA, getEigenVectors)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::Matrix3f& eigenVectors = (*self)->getEigenVectors();
  copyMatrix(eigenVectors, output);
}

PCLIMP(void, PCA, getEigenValues)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::Vector3f& eigenValues = (*self)->getEigenValues();
  copyMatrix(eigenValues, output);
}

PCLIMP(void, PCA, getCoefficients)(PCA_ptr *self, THFloatTensor* output)
{
  Eigen::MatrixXf& coefficients =(*self)->getCoefficients();
  viewMatrix(coefficients, output);
}

PCLIMP(void, PCA, projectCloud)(PCA_ptr *self, PointCloud_ptr *input, PointCloud_ptr *output)
{
  (*self)->project(**input, **output);
}

PCLIMP(void, PCA, reconstructCloud)(PCA_ptr *self, PointCloud_ptr *input, PointCloud_ptr *output)
{
  (*self)->reconstruct(**input, **output);
}

#undef PCA_ptr
#undef PointCloud_ptr
