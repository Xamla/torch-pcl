#define _PointT PointXYZRGBANormal
#define TYPE_KEY _XYZRGBANormal_
#define _PointT_HAS_NORMALS
#include "utils.h"
#include "generic.cpp"

std::ostream& operator << (std::ostream& os, const PointXYZRGBANormal& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.r << ", " << p.g << ", " << p.b << ", " << p.a << " - " << p.curvature << ")";
  return (os);
}

// explicitely instantiate impls
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/shadowpoints.hpp>
#include <pcl/filters/impl/normal_space.hpp>
#include <pcl/filters/impl/normal_refinement.hpp>
#include <pcl/filters/impl/frustum_culling.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/filters/impl/random_sample.hpp>
#include <pcl/filters/impl/median_filter.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>

#include <pcl/common/impl/pca.hpp>
#include <pcl/common/impl/io.hpp>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/brute_force.hpp>
#include <pcl/search/impl/flann_search.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_search.hpp>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/registration/impl/icp_nl.hpp>
#include <pcl/registration/impl/incremental_registration.hpp>

template class PCL_EXPORTS pcl::PCLBase<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::PointCloud<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::PCA<PointXYZRGBANormal>;

// filters
template PCL_EXPORTS void pcl::removeNaNFromPointCloud<PointXYZRGBANormal>(const pcl::PointCloud<PointXYZRGBANormal>&, pcl::PointCloud<PointXYZRGBANormal>&, std::vector<int>&);
template PCL_EXPORTS void pcl::removeNaNNormalsFromPointCloud<PointXYZRGBANormal>(const pcl::PointCloud<PointXYZRGBANormal>&, pcl::PointCloud<PointXYZRGBANormal>&, std::vector<int>&);
template PCL_EXPORTS void pcl::removeNaNFromPointCloud<PointXYZRGBANormal>(const pcl::PointCloud<PointXYZRGBANormal>&, std::vector<int>&);

template class PCL_EXPORTS pcl::ExtractIndices<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::ShadowPoints<PointXYZRGBANormal, pcl::Normal>;
template class PCL_EXPORTS pcl::NormalSpaceSampling<PointXYZRGBANormal, pcl::Normal>;
template class PCL_EXPORTS pcl::NormalRefinement<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::FrustumCulling<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::PassThrough<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::CropBox<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::VoxelGrid<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::StatisticalOutlierRemoval<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::RandomSample<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::MedianFilter<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::RadiusOutlierRemoval<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::ConditionalRemoval<PointXYZRGBANormal>;

// search
template class PCL_EXPORTS pcl::search::Search<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::search::OrganizedNeighbor<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::search::BruteForce<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::search::FlannSearch<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::search::KdTree<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::search::Octree<PointXYZRGBANormal>;

template class PCL_EXPORTS pcl::octree::OctreeBase<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::octree::OctreePointCloud<PointXYZRGBANormal, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty > >;
template class PCL_EXPORTS pcl::octree::OctreePointCloud<PointXYZRGBANormal, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty, pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty > >;
template class PCL_EXPORTS pcl::octree::OctreePointCloud<PointXYZRGBANormal, pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty > >;
template class PCL_EXPORTS pcl::octree::OctreePointCloud<PointXYZRGBANormal, pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty, pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty > >;
template class PCL_EXPORTS pcl::octree::OctreePointCloud<PointXYZRGBANormal, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeBase<pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty > >;
template class PCL_EXPORTS pcl::octree::OctreePointCloud<PointXYZRGBANormal, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty, pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty > >;
template class PCL_EXPORTS pcl::octree::OctreePointCloudSearch<PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::KdTreeFLANN<PointXYZRGBANormal>;

// ICP
template class PCL_EXPORTS pcl::IterativeClosestPoint<PointXYZRGBANormal, PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::IterativeClosestPointNonLinear<PointXYZRGBANormal, PointXYZRGBANormal>;
template class PCL_EXPORTS pcl::registration::IncrementalRegistration<PointXYZRGBANormal, float>;
