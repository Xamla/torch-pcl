#include <pcl/registration/sample_consensus_prerejective.h>

#define SampleConsensusPrerejective_ptr pcl::SampleConsensusPrerejective<_PointT, _PointT, pcl::FPFHSignature33>::Ptr
#define PointCloud_ptr pcl::PointCloud<_PointT>::Ptr
#define FeatureCloud_ptr pcl::PointCloud<pcl::FPFHSignature33>::Ptr

PCLIMP(SampleConsensusPrerejective_ptr *, SampleConsensusPrerejective, new)()
{
  return new SampleConsensusPrerejective_ptr(new pcl::SampleConsensusPrerejective<_PointT, _PointT, pcl::FPFHSignature33>());
}

PCLIMP(void, SampleConsensusPrerejective, delete)(SampleConsensusPrerejective_ptr *self)
{
  delete self;
}

PCLIMP(void, SampleConsensusPrerejective, setInputSource)(SampleConsensusPrerejective_ptr *self, PointCloud_ptr *source_cloud)
{
  (*self)->setInputSource(*source_cloud);
}

PCLIMP(void, SampleConsensusPrerejective, setSourceFeatures)(SampleConsensusPrerejective_ptr *self, FeatureCloud_ptr* source_features)
{
  (*self)->setSourceFeatures(*source_features);
}

PCLIMP(void, SampleConsensusPrerejective, setInputTarget)(SampleConsensusPrerejective_ptr *self, PointCloud_ptr *target_cloud)
{
  (*self)->setInputTarget(*target_cloud);
}

PCLIMP(void, SampleConsensusPrerejective, setTargetFeatures)(SampleConsensusPrerejective_ptr *self, FeatureCloud_ptr* target_features)
{
  (*self)->setTargetFeatures(*target_features);
}

PCLIMP(void, SampleConsensusPrerejective, setMaximumIterations)(SampleConsensusPrerejective_ptr *self, int max_iterations)
{
  (*self)->setMaximumIterations(max_iterations);
}

PCLIMP(int, SampleConsensusPrerejective, getMaximumIterations)(SampleConsensusPrerejective_ptr *self)
{
  return (*self)->getMaximumIterations();
}

PCLIMP(void, SampleConsensusPrerejective, setNumberOfSamples)(SampleConsensusPrerejective_ptr *self, int nr_samples)
{
  (*self)->setNumberOfSamples(nr_samples);
}

PCLIMP(int, SampleConsensusPrerejective, getNumberOfSamples)(SampleConsensusPrerejective_ptr *self)
{
  return (*self)->getNumberOfSamples();
}

PCLIMP(void, SampleConsensusPrerejective, setCorrespondenceRandomness)(SampleConsensusPrerejective_ptr *self, int k)
{
  (*self)->setCorrespondenceRandomness(k);
}

PCLIMP(int, SampleConsensusPrerejective, getCorrespondenceRandomness)(SampleConsensusPrerejective_ptr *self)
{
  return (*self)->getCorrespondenceRandomness();
}

PCLIMP(void, SampleConsensusPrerejective, setSimilarityThreshold)(SampleConsensusPrerejective_ptr *self, float similarity_threshold)
{
  (*self)->setSimilarityThreshold(similarity_threshold);
}

PCLIMP(float, SampleConsensusPrerejective, getSimilarityThreshold)(SampleConsensusPrerejective_ptr *self)
{
  return (*self)->getSimilarityThreshold();
}

PCLIMP(void, SampleConsensusPrerejective, setInlierFraction)(SampleConsensusPrerejective_ptr *self, float inlier_fraction)
{
  (*self)->setInlierFraction(inlier_fraction);
}

PCLIMP(float, SampleConsensusPrerejective, getInlierFraction)(SampleConsensusPrerejective_ptr *self)
{
  return (*self)->getInlierFraction();
}

PCLIMP(void, SampleConsensusPrerejective, getInliers)(SampleConsensusPrerejective_ptr *self, Indices_ptr *indices)
{
  std::vector<int>& out = **indices;
  out = (*self)->getInliers();
}

#undef SampleConsensusPrerejective_ptr
#undef PointCloud_ptr
