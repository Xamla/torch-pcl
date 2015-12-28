local ffi = require 'ffi'

local pcl = {}
pcl.NULL = ffi.NULL or nil

local PCL_POINT4D = "union __attribute__((aligned(16))) { struct { float x; float y; float z; }; float data[4]; };"
local PCL_NORMAL4D = "union __attribute__((aligned(16))) { struct { float normal_x; float normal_y; float normal_z; }; float normal[3]; float data_n[4]; };"
local PCL_RGB = "union { uint32_t rgba; union { struct { uint8_t b; uint8_t g; uint8_t r; uint8_t a; }; float rgb; }; };"

local cdef = "enum NormType { L1, L2_SQR, L2, LINF, JM, B, SUBLINEAR, CS, DIV, PF, K, KL, HIK }; \z
typedef struct RGB { "..PCL_RGB.." } RGB; \z
typedef struct PointXY { float x; float y; } PointXY; \z
typedef struct PointUV { float u; float v; } PointUV; \z
typedef struct InterestPoint { "..PCL_POINT4D.." union { struct { float strength; }; float data_c[4]; }; } InterestPoint; \z
typedef struct PointXYZ { "..PCL_POINT4D.."} PointXYZ; \z
typedef struct PointXYZI { "..PCL_POINT4D.." union { struct { float intensity; }; float data_c[4]; }; } PointXYZI; \z
typedef struct PointXYZL { "..PCL_POINT4D.." uint32_t label; } PointXYZL; \z
typedef struct PointXYZRGBA { "..PCL_POINT4D..PCL_RGB.." } PointXYZRGBA; \z
typedef struct PointXYZRGBL { "..PCL_POINT4D..PCL_RGB.." uint32_t label; } PointXYZRGBL; \z
typedef struct Normal { "..PCL_NORMAL4D.." union { struct { float curvature; }; float data_c[4]; }; } Normal; \z
typedef struct Axis { "..PCL_POINT4D.." } Axis; \z
typedef struct PointNormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float curvature; }; float data_c[4]; }; } PointNormal; \z
typedef struct PointNormal PointXYZNormal;\z
typedef struct PointXYZRGBANormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { "..PCL_RGB.." float curvature; }; float data_c[4]; }; } PointXYZRGBANormal; \z
typedef struct PointXYZINormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float intensity; float curvature; }; float data_c[4]; }; } PointXYZINormal; \z
typedef struct _PointsBuffer { THFloatStorage* storage; uint32_t width, height, dim; } _PointsBuffer;" ..
[[
typedef struct OpenNI2CameraParameters { double focal_length_x; double focal_length_y; double principal_point_x; double principal_point_y; } OpenNI2CameraParameters;
typedef struct PointCloud_XYZ {} PointCloud_XYZ;
typedef struct PointCloud_XYZI {} PointCloud_XYZI;
typedef struct PointCloud_XYZRGBA {} PointCloud_XYZRGBA;
typedef struct PointCloud_XYZNormal {} PointCloud_XYZNormal;
typedef struct PointCloud_XYZINormal {} PointCloud_XYZINormal;
typedef struct PointCloud_XYZRGBANormal {} PointCloud_XYZRGBANormal;
typedef struct PointCloud_Normal {} PointCloud_Normal;

void* pcl_CloudViewer_new(const char *window_name);
void pcl_CloudViewer_delete(void *self);
bool pcl_CloudViewer_wasStopped(void *self, int millis_to_wait);

void pcl_Primitive_XYZ_createSphere(PointCloud_XYZ *output, double radius, double thetaRes, double phiRes, int samples, float resolution);
void pcl_Primitive_XYZ_createCube(PointCloud_XYZ *output, double x, double y, double z, int samples, float resolution);
void pcl_Primitive_XYZ_createCylinder(PointCloud_XYZ *output, double height, double radius, int facets, int samples, float resolution);
void pcl_Primitive_XYZ_createCone(PointCloud_XYZ *output, double height, double radius, int facets, int samples, float resolution);
void pcl_Primitive_XYZ_createPlatonicSolid(PointCloud_XYZ *output, int solidType, int samples, float resolution);
void pcl_Primitive_XYZ_createPlane(PointCloud_XYZ *output, double x1, double y1, double z1, double x2, double y2, double z2, int samples, float resolution);
void pcl_Primitive_XYZ_createDisk(PointCloud_XYZ *output, double innerRadius, double outerRadius, int radialResolution,int samples, float resolution);

typedef struct Indices {} Indices;
Indices* pcl_Indices_new();
Indices* pcl_Indices_clone(Indices *self);
void pcl_Indices_delete(Indices *self);
unsigned int pcl_Indices_size(Indices *self);
unsigned int pcl_Indices_capacity(Indices *self);
void pcl_Indices_reserve(Indices *self, size_t capacity);
void pcl_Indices_append(Indices *self, Indices *source);
void pcl_Indices_insertMany(Indices *self, Indices *source, size_t src_offset, size_t dst_offset, size_t count);
void pcl_Indices_viewAsTensor(Indices* self, THIntTensor* tensor);
void pcl_Indices_copyToTensor(Indices* self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count);
void pcl_Indices_copyFromTensor(Indices* self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count);
void pcl_Indices_insertFromTensor(Indices* self, THIntTensor* tensor, size_t src_offset, size_t dst_offset, size_t count);
int pcl_Indices_getAt(Indices *self, size_t pos);
void pcl_Indices_setAt(Indices *self, size_t pos, int value);
void pcl_Indices_push_back(Indices *self, int value);
int pcl_Indices_pop_back(Indices *self);
void pcl_Indices_clear(Indices *self);
void pcl_Indices_insert(Indices *self, size_t pos, size_t n, int value);
void pcl_Indices_erase(Indices *self, size_t begin, size_t end);
]]
ffi.cdef(cdef)

local pcl_PointCloud_declaration = [[
PointCloud_TYPE_KEY* pcl_PointCloud_TYPE_KEY_new(uint32_t width, uint32_t height);
PointCloud_TYPE_KEY* pcl_PointCloud_TYPE_KEY_clone(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_delete(PointCloud_TYPE_KEY *self);
uint32_t pcl_PointCloud_TYPE_KEY_getWidth(PointCloud_TYPE_KEY *self);
uint32_t pcl_PointCloud_TYPE_KEY_getHeight(PointCloud_TYPE_KEY *self);
bool pcl_PointCloud_TYPE_KEY_getIsDense(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_setIsDense(PointCloud_TYPE_KEY *self, bool value);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at1D(PointCloud_TYPE_KEY *self, int n);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at2D(PointCloud_TYPE_KEY *self, int column, int row);
void pcl_PointCloud_TYPE_KEY_clear(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_reserve(PointCloud_TYPE_KEY *self, size_t n);
uint32_t pcl_PointCloud_TYPE_KEY_size(PointCloud_TYPE_KEY *self);
bool pcl_PointCloud_TYPE_KEY_empty(PointCloud_TYPE_KEY *self);
bool pcl_PointCloud_TYPE_KEY_isOrganized(PointCloud_TYPE_KEY* self);
void pcl_PointCloud_TYPE_KEY_push_back(PointCloud_TYPE_KEY* self, const PointTYPE_KEY& pt);
void pcl_PointCloud_TYPE_KEY_insert(PointCloud_TYPE_KEY* self, size_t position, size_t n, const PointTYPE_KEY& pt);
void pcl_PointCloud_TYPE_KEY_erase(PointCloud_TYPE_KEY* self, size_t begin, size_t end);
_PointsBuffer pcl_PointCloud_TYPE_KEY_points(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_add(PointCloud_TYPE_KEY *self, PointCloud_TYPE_KEY *other);
THFloatStorage *pcl_PointCloud_TYPE_KEY_sensorOrigin(PointCloud_TYPE_KEY *self);
THFloatStorage *pcl_PointCloud_TYPE_KEY_sensorOrientation(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_transform(PointCloud_TYPE_KEY *self, THFloatTensor *mat, PointCloud_TYPE_KEY *output);
void pcl_PointCloud_TYPE_KEY_transformWithNormals(PointCloud_TYPE_KEY *self, THFloatTensor *mat, PointCloud_TYPE_KEY *output);
void pcl_PointCloud_TYPE_KEY_getMinMax3D(PointCloud_TYPE_KEY *self, PointTYPE_KEY& min, PointTYPE_KEY& max);
void pcl_PointCloud_TYPE_KEY_compute3DCentroid(PointCloud_TYPE_KEY *self, THFloatTensor *output);
void pcl_PointCloud_TYPE_KEY_computeCovarianceMatrix(PointCloud_TYPE_KEY *self, THFloatTensor *centroid, THFloatTensor *output);
void pcl_PointCloud_TYPE_KEY_fromPCLPointCloud2(void *cloud, void *msg);
void pcl_PointCloud_TYPE_KEY_toPCLPointCloud2(void *cloud, void *msg);
int pcl_PointCloud_TYPE_KEY_loadPCDFile(PointCloud_TYPE_KEY *cloud, const char *fn);
int pcl_PointCloud_TYPE_KEY_savePCDFile(PointCloud_TYPE_KEY *cloud, const char *fn, bool binary);
int pcl_PointCloud_TYPE_KEY_loadPLYFile(PointCloud_TYPE_KEY *cloud, const char *fn);
int pcl_PointCloud_TYPE_KEY_savePLYFile(PointCloud_TYPE_KEY *cloud, const char *fn, bool binary);
int pcl_PointCloud_TYPE_KEY_loadOBJFile(PointCloud_TYPE_KEY *cloud, const char *fn);
void pcl_PointCloud_TYPE_KEY_savePNGFile(PointCloud_TYPE_KEY *cloud, const char *fn, const char* field_name);
int pcl_PointCloud_TYPE_KEY_readXYZfloat(PointCloud_TYPE_KEY *cloud, struct THFloatTensor *output);
void pcl_PointCloud_TYPE_KEY_copyXYZ(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZ *cloud_out);
void pcl_PointCloud_TYPE_KEY_copyXYZI(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZI *cloud_out);
void pcl_PointCloud_TYPE_KEY_copyXYZRGBA(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZRGBA *cloud_out);
void pcl_PointCloud_TYPE_KEY_copyXYZNormal(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZNormal *cloud_out);
void pcl_PointCloud_TYPE_KEY_copyXYZINormal(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZINormal *cloud_out);
void pcl_PointCloud_TYPE_KEY_copyXYZRGBANormal(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZRGBANormal *cloud_out);
]]

local generic_declarations = [[
void pcl_CloudViewer_TYPE_KEY_showCloud(void *self, PointCloud_TYPE_KEY *cloud, const char *cloudname);

void pcl_Filter_TYPE_KEY_extractIndices(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_shadowPoints_Indices(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_Normal *normals, Indices *output, float threshold, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_shadowPoints_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_Normal *normals, PointCloud_TYPE_KEY *output, float threshold, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_removeNaNFromPointCloud(PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output, Indices *indices);
void pcl_Filter_TYPE_KEY_removeNaNNormalsFromPointCloud(PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output, Indices *indices);
void pcl_Filter_TYPE_KEY_normalSpaceSampling_Indices(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_Normal *normals, Indices *output, unsigned int samples, unsigned int binsx, unsigned int binsy, unsigned int binsz);
void pcl_Filter_TYPE_KEY_normalSpaceSampling_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_Normal *normals, PointCloud_TYPE_KEY *output, unsigned int samples, unsigned int binsx, unsigned int binsy, unsigned int binsz);
void pcl_Filter_TYPE_KEY_normalRefinement(PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output, int k, int max_iterations, float convergence_threshold);
void pcl_Filter_TYPE_KEY_frustumCulling_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, THFloatTensor *cameraPose, float hfov, float vfov, float np_dist, float fp_dist, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_frustumCulling_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, THFloatTensor *cameraPose, float hfov, float vfov, float np_dist, float fp_dist, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_passThrough_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, const char* fieldName, float min, float max, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_passThrough_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, const char* fieldName, float min, float max, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_cropBox_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, THFloatTensor *transform, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_cropBox_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, THFloatTensor *transform, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_cropSphere_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_cropSphere_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_voxelGrid(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, float lx, float ly, float lz);
void pcl_Filter_TYPE_KEY_statisticalOutlierRemoval_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, int meanK, double stddevMulThresh, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_statisticalOutlierRemoval_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, int meanK, double stddevMulThresh, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_randomSample_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, unsigned int count);
void pcl_Filter_TYPE_KEY_randomSample_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, unsigned int count);
void pcl_Filter_TYPE_KEY_medianFilter(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, int windowSize);
void pcl_Filter_TYPE_KEY_radiusOutlierRemoval(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, double radius, int minNeighbors, bool negative, Indices *removed_indices);
int pcl_Filter_TYPE_KEY_voxelHistogram(PointCloud_TYPE_KEY *input, THFloatTensor *output, int w, int h, int t, float voxelSize, float originX, float originY, float originZ, bool center);

typedef struct {} PCA_TYPE_KEY;
PCA_TYPE_KEY* pcl_PCA_TYPE_KEY_new(bool basis_only);
PCA_TYPE_KEY* pcl_PCA_TYPE_KEY_clone(PCA_TYPE_KEY *self);
void pcl_PCA_TYPE_KEY_delete(PCA_TYPE_KEY *self);
void pcl_PCA_TYPE_KEY_setInputCloud(PCA_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_PCA_TYPE_KEY_setIndices(PCA_TYPE_KEY *self, Indices *indices);
void pcl_PCA_TYPE_KEY_getMean(PCA_TYPE_KEY *self, struct THFloatTensor* output);
void pcl_PCA_TYPE_KEY_getEigenVectors(PCA_TYPE_KEY *self, struct THFloatTensor *output);
void pcl_PCA_TYPE_KEY_getEigenValues(PCA_TYPE_KEY *self, struct THFloatTensor *output);
void pcl_PCA_TYPE_KEY_getCoefficients(PCA_TYPE_KEY *self, struct THFloatTensor* output);
void pcl_PCA_TYPE_KEY_projectCloud(PCA_TYPE_KEY *self, PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output);
void pcl_PCA_TYPE_KEY_reconstructCloud(PCA_TYPE_KEY *self, PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output);

typedef struct {} ICP_TYPE_KEY;
ICP_TYPE_KEY* pcl_ICP_TYPE_KEY_new();
void pcl_ICP_TYPE_KEY_delete(ICP_TYPE_KEY *self);
void pcl_ICP_TYPE_KEY_setInputSource(ICP_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_ICP_TYPE_KEY_setInputTarget(ICP_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_ICP_TYPE_KEY_setMaxCorrespondenceDistance(ICP_TYPE_KEY *self, double distance);
void pcl_ICP_TYPE_KEY_setMaximumIterations(ICP_TYPE_KEY *self, int count);
void pcl_ICP_TYPE_KEY_setTransformationEpsilon(ICP_TYPE_KEY *self, double epsilon);
void pcl_ICP_TYPE_KEY_setEuclideanFitnessEpsilon(ICP_TYPE_KEY *self, double epsilon);
void pcl_ICP_TYPE_KEY_getFinalTransformation(ICP_TYPE_KEY *self, THFloatTensor* output);
double pcl_ICP_TYPE_KEY_getFitnessScore(ICP_TYPE_KEY *self, double max_range);
void pcl_ICP_TYPE_KEY_align(ICP_TYPE_KEY *self, PointCloud_TYPE_KEY* output, void* guess);
void pcl_ICP_TYPE_KEY_addDistanceRejector(ICP_TYPE_KEY *self, float max_distance);
void pcl_ICP_TYPE_KEY_addSurfaceNormalRejector(ICP_TYPE_KEY *self, float threshold);
void pcl_ICP_TYPE_KEY_addRANSACRejector(ICP_TYPE_KEY *self, double inlier_threshold, int max_iterations);
void pcl_ICP_TYPE_KEY_addOneToOneRejector(ICP_TYPE_KEY *self);
void pcl_ICP_TYPE_KEY_addTrimmedRejector(ICP_TYPE_KEY *self, float overlap_ratio, unsigned int min_correspondences);

typedef struct {} ICPNL_TYPE_KEY;
ICPNL_TYPE_KEY* pcl_ICPNL_TYPE_KEY_new();
void pcl_ICPNL_TYPE_KEY_delete(ICPNL_TYPE_KEY *self);
void pcl_ICPNL_TYPE_KEY_setInputSource(ICPNL_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_ICPNL_TYPE_KEY_setInputTarget(ICPNL_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_ICPNL_TYPE_KEY_setMaxCorrespondenceDistance(ICPNL_TYPE_KEY *self, double distance);
void pcl_ICPNL_TYPE_KEY_setMaximumIterations(ICPNL_TYPE_KEY *self, int count);
void pcl_ICPNL_TYPE_KEY_setTransformationEpsilon(ICPNL_TYPE_KEY *self, double epsilon);
void pcl_ICPNL_TYPE_KEY_setEuclideanFitnessEpsilon(ICPNL_TYPE_KEY *self, double epsilon);
void pcl_ICPNL_TYPE_KEY_getFinalTransformation(ICPNL_TYPE_KEY *self, THFloatTensor* output);
double pcl_ICPNL_TYPE_KEY_getFitnessScore(ICPNL_TYPE_KEY *self, double max_range);
void pcl_ICPNL_TYPE_KEY_align(ICPNL_TYPE_KEY *self, PointCloud_TYPE_KEY* output, void* guess);
void pcl_ICPNL_TYPE_KEY_addDistanceRejector(ICPNL_TYPE_KEY *self, float max_distance);
void pcl_ICPNL_TYPE_KEY_addSurfaceNormalRejector(ICPNL_TYPE_KEY *self, float threshold);
void pcl_ICPNL_TYPE_KEY_addRANSACRejector(ICPNL_TYPE_KEY *self, double inlier_threshold, int max_iterations);
void pcl_ICPNL_TYPE_KEY_addOneToOneRejector(ICPNL_TYPE_KEY *self);
void pcl_ICPNL_TYPE_KEY_addTrimmedRejector(ICPNL_TYPE_KEY *self, float overlap_ratio, unsigned int min_correspondences);

typedef struct {} IncrementalRegistration_TYPE_KEY;
void* pcl_IncrementalRegistration_TYPE_KEY_new();
void pcl_IncrementalRegistration_TYPE_KEY_delete(IncrementalRegistration_TYPE_KEY *self);
void pcl_IncrementalRegistration_TYPE_KEY_setICP(IncrementalRegistration_TYPE_KEY *self, ICP_TYPE_KEY *icp);
void pcl_IncrementalRegistration_TYPE_KEY_setICPNL(IncrementalRegistration_TYPE_KEY *self, ICPNL_TYPE_KEY *icpnl);
void pcl_IncrementalRegistration_TYPE_KEY_reset(IncrementalRegistration_TYPE_KEY *self);
bool pcl_IncrementalRegistration_TYPE_KEY_registerCloud(IncrementalRegistration_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud, THFloatTensor* delta_estimate);
void pcl_IncrementalRegistration_TYPE_KEY_getDeltaTransform(IncrementalRegistration_TYPE_KEY *self, THFloatTensor* output);
void pcl_IncrementalRegistration_TYPE_KEY_getAbsoluteTransform(IncrementalRegistration_TYPE_KEY *self, THFloatTensor* output);

typedef struct {} KdTreeFLANN_TYPE_KEY;
KdTreeFLANN_TYPE_KEY* pcl_KdTreeFLANN_TYPE_KEY_new(bool sorted);
KdTreeFLANN_TYPE_KEY* pcl_KdTreeFLANN_TYPE_KEY_clone(KdTreeFLANN_TYPE_KEY *self);
void pcl_KdTreeFLANN_TYPE_KEY_delete(KdTreeFLANN_TYPE_KEY *self);
void pcl_KdTreeFLANN_TYPE_KEY_setInputCloud(KdTreeFLANN_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud, Indices *indices);
float pcl_KdTreeFLANN_TYPE_KEY_getEpsilon(KdTreeFLANN_TYPE_KEY *self);
void pcl_KdTreeFLANN_TYPE_KEY_setEpsilon(KdTreeFLANN_TYPE_KEY *self, float value);
void pcl_KdTreeFLANN_TYPE_KEY_setMinPts(KdTreeFLANN_TYPE_KEY *self, int value);
int pcl_KdTreeFLANN_TYPE_KEY_getMinPts(KdTreeFLANN_TYPE_KEY *self);
void pcl_KdTreeFLANN_TYPE_KEY_setSortedResults(KdTreeFLANN_TYPE_KEY *self, bool value);
void pcl_KdTreeFLANN_TYPE_KEY_assign(KdTreeFLANN_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *other);
int pcl_KdTreeFLANN_TYPE_KEY_nearestKSearch(KdTreeFLANN_TYPE_KEY *self, const PointTYPE_KEY &point, int k, Indices *indices, THFloatTensor *squaredDistances);
int pcl_KdTreeFLANN_TYPE_KEY_radiusSearch(KdTreeFLANN_TYPE_KEY *self, const PointTYPE_KEY &point, double radius, Indices *indices, THFloatTensor *squaredDistances, unsigned int max_nn);

typedef struct {} OctreePointCloudSearch_TYPE_KEY;
OctreePointCloudSearch_TYPE_KEY* pcl_OctreePointCloudSearch_TYPE_KEY_new(double resolution);
void pcl_OctreePointCloudSearch_TYPE_KEY_delete(OctreePointCloudSearch_TYPE_KEY *self);
double pcl_OctreePointCloudSearch_TYPE_KEY_getResolution(OctreePointCloudSearch_TYPE_KEY *self);
double pcl_OctreePointCloudSearch_TYPE_KEY_getEpsilon(OctreePointCloudSearch_TYPE_KEY *self);
void pcl_OctreePointCloudSearch_TYPE_KEY_setEpsilon(OctreePointCloudSearch_TYPE_KEY *self, double value);
void pcl_OctreePointCloudSearch_TYPE_KEY_setInputCloud(OctreePointCloudSearch_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud, Indices *indices);
void pcl_OctreePointCloudSearch_TYPE_KEY_addPointsFromInputCloud(OctreePointCloudSearch_TYPE_KEY *self);
void pcl_OctreePointCloudSearch_TYPE_KEY_addPointToCloud(OctreePointCloudSearch_TYPE_KEY *self, const PointTYPE_KEY &point, PointCloud_TYPE_KEY *cloud);
bool pcl_OctreePointCloudSearch_TYPE_KEY_isVoxelOccupiedAtPoint(OctreePointCloudSearch_TYPE_KEY *self, const PointTYPE_KEY &point);
void pcl_OctreePointCloudSearch_TYPE_KEY_deleteTree(OctreePointCloudSearch_TYPE_KEY *self);
void pcl_OctreePointCloudSearch_TYPE_KEY_setMaxVoxelIndex(OctreePointCloudSearch_TYPE_KEY *self, unsigned int value);
void pcl_OctreePointCloudSearch_TYPE_KEY_setTreeDepth(OctreePointCloudSearch_TYPE_KEY *self, unsigned int value);
unsigned int pcl_OctreePointCloudSearch_TYPE_KEY_getTreeDepth(OctreePointCloudSearch_TYPE_KEY *self);
unsigned int pcl_OctreePointCloudSearch_TYPE_KEY_getLeafCount(OctreePointCloudSearch_TYPE_KEY *self);
unsigned int pcl_OctreePointCloudSearch_TYPE_KEY_getBranchCount(OctreePointCloudSearch_TYPE_KEY *self);
int pcl_OctreePointCloudSearch_TYPE_KEY_nearestKSearch(OctreePointCloudSearch_TYPE_KEY *self, const PointTYPE_KEY &point, int k, Indices *indices, THFloatTensor *squaredDistances);
int pcl_OctreePointCloudSearch_TYPE_KEY_radiusSearch(OctreePointCloudSearch_TYPE_KEY *self, const PointTYPE_KEY &point, double radius, Indices *indices, THFloatTensor *squaredDistances, unsigned int max_nn);

typedef struct {} NormalEstimation_TYPE_KEY;
NormalEstimation_TYPE_KEY* pcl_NormalEstimation_TYPE_KEY_new();
void pcl_NormalEstimation_TYPE_KEY_delete(NormalEstimation_TYPE_KEY *self);
void pcl_NormalEstimation_TYPE_KEY_setInputCloud(NormalEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY* cloud);
void pcl_NormalEstimation_TYPE_KEY_setIndices(NormalEstimation_TYPE_KEY *self, Indices *indices);
void pcl_NormalEstimation_TYPE_KEY_getViewPoint(NormalEstimation_TYPE_KEY *self, THFloatTensor* out_pt);
void pcl_NormalEstimation_TYPE_KEY_setViewPoint(NormalEstimation_TYPE_KEY *self, THFloatTensor* pt);
void pcl_NormalEstimation_TYPE_KEY_useSensorOriginAsViewPoint(NormalEstimation_TYPE_KEY *self);
void pcl_NormalEstimation_TYPE_KEY_setSearchMethod_Octree(NormalEstimation_TYPE_KEY *self, OctreePointCloudSearch_TYPE_KEY *octree);
void pcl_NormalEstimation_TYPE_KEY_setSearchMethod_KdTree(NormalEstimation_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *kdtree);
void pcl_NormalEstimation_TYPE_KEY_setKSearch(NormalEstimation_TYPE_KEY *self, int k);
int pcl_NormalEstimation_TYPE_KEY_getKSearch(NormalEstimation_TYPE_KEY *self);
void pcl_NormalEstimation_TYPE_KEY_setRadiusSearch(NormalEstimation_TYPE_KEY *self, double radius);
double pcl_NormalEstimation_TYPE_KEY_getRadiusSearch(NormalEstimation_TYPE_KEY *self);
void pcl_NormalEstimation_TYPE_KEY_compute(NormalEstimation_TYPE_KEY *self, PointCloud_Normal* output);
void pcl_NormalEstimation_TYPE_KEY_setNumberOfThreads(NormalEstimation_TYPE_KEY *self, unsigned int num_threads);

void* pcl_OpenNI2Stream_TYPE_KEY_new(const char* device_id, int max_backlog);
void pcl_OpenNI2Stream_TYPE_KEY_delete(void* self);
void pcl_OpenNI2Stream_TYPE_KEY_start(void* self);
void pcl_OpenNI2Stream_TYPE_KEY_stop(void* self);
void* pcl_OpenNI2Stream_TYPE_KEY_read(void* self, int timeout_milliseconds);
void pcl_OpenNI2Stream_TYPE_KEY_getRGBCameraIntrinsics(void *self, OpenNI2CameraParameters& p);
void pcl_OpenNI2Stream_TYPE_KEY_setRGBCameraIntrinsics(void *self, const OpenNI2CameraParameters& p);
void pcl_OpenNI2Stream_TYPE_KEY_getDepthCameraIntrinsics(void *self, OpenNI2CameraParameters& p);
void pcl_OpenNI2Stream_TYPE_KEY_setDepthCameraIntrinsics(void *self, const OpenNI2CameraParameters& p);
const char* pcl_OpenNI2Stream_TYPE_KEY_getName(void *self);
float pcl_OpenNI2Stream_TYPE_KEY_getFramesPerSecond(void *self);
]]

local supported_keys = { 'XYZ', 'XYZI', 'XYZRGBA', 'XYZNormal', 'XYZINormal', 'XYZRGBANormal' }
for i,v in ipairs(supported_keys) do
  local specialized = string.gsub(pcl_PointCloud_declaration, 'TYPE_KEY', v)
  ffi.cdef(specialized)
  specialized = string.gsub(generic_declarations, 'TYPE_KEY', v)
  ffi.cdef(specialized)
end

local function add_normal_declarations()
  local specialized = string.gsub(pcl_PointCloud_declaration, 'PointTYPE_KEY', 'Normal')
  specialized = string.gsub(specialized, 'TYPE_KEY', 'Normal')
  ffi.cdef(specialized)
end
add_normal_declarations()


local specialized_declarations = 
[[
void pcl_PointCloud_XYZRGBA_readRGBAfloat_readRGBAfloat(void *cloud, struct THFloatTensor *output);
void pcl_PointCloud_XYZRGBA_readRGBAbyte(void *cloud, struct THByteTensor *output);

void pcl_PointCloud_XYZ_addNormals(PointCloud_XYZ *self, PointCloud_Normal *normals, PointCloud_XYZNormal *output);
void pcl_PointCloud_XYZI_addNormals(PointCloud_XYZI *self, PointCloud_Normal *normals, PointCloud_XYZINormal *output);
void pcl_PointCloud_XYZRGBA_addNormals(PointCloud_XYZRGBA*self, PointCloud_Normal *normals, PointCloud_XYZRGBANormal *output);
]]
ffi.cdef(specialized_declarations)

pcl.lib = ffi.load(package.searchpath('libpcl', package.cpath))

local pointTypeNames = { 
  'PointXYZ',             -- float x, y, z;
  'PointXYZI',            -- float x, y, z, intensity;
  'PointXYZRGBA',         -- float x, y, z; uint32_t rgb;
  'PointXY',              -- float x, y;
  'PointNormal',          -- float x, y, z; float normal[3], curvature;
  'Normal',               -- float normal[3], curvature;
  'PointXYZNormal',       -- alias for PointNormal
  'PointXYZRGBANormal',    -- float x, y, z, rgb, normal[3], curvature;
  'PointXYZINormal'       -- float x, y, z, intensity, normal[3], curvature;
}

local nameByType = {}
pcl.pointTypeByName = {}

function pcl.pointType(pointType)
  if type(pointType) == 'string' then
    local t = pcl.pointTypeByName[string.lower(pointType)]
    if not t then
      error('Invalid point type: \'' .. pointType .. '\' specified.')
    end
    pointType = t
  end
  return pointType or pcl.PointXYZ  
end

for i,n in ipairs(pointTypeNames) do
  local t = ffi.typeof(n) 
  pcl[n] = t
  nameByType[t] = n
  pcl.pointTypeByName[string.lower(n)] = t
  n = string.gsub(n, 'Point', '')
  pcl.pointTypeByName[string.lower(n)] = t
end

function pcl.isPointType(t)
  return nameByType[t] ~= nil
end

local function createpairs(fields)
  return function(self)
    local k = nil
    return function()
      local _
      k, n = next(fields, k)
      local v = self[k]
      return n,v
    end
  end
end

pcl.metatype = {}

-- compare fields
local function eq(a,b)
  local l
  if torch.isTensor(b) then
    if b:dim() ~= 1 then
      error('1D tensor expected')
    end
    l = b:size(1)
  else
    l = #b
  end
  
  if not l then return false end
  
  for i=1,l do
    if a[i] ~= b[i] then
      return false
    end
  end
  
  return true
end

local function set(dst, v)
  if type(v) == 'cdata' and ffi.istype(v, dst) then
    ffi.copy(dst, v, ffi.sizeof(dst))
  elseif type(v) == 'table' then
    for i=1,#v do
      dst[i] = v[i]
    end
  elseif torch.isTensor(v) then
    if v:dim() ~= 1 then
      error('1D tensor expected')
    end
    for i=1,v:size(1) do
      dst[i] = v[i]
    end
  end
  return dst
end

local function len(self)
  return ffi.sizeof(self) / ffi.sizeof('float')
end

local function totensor(self)
  local t = torch.FloatTensor(#self)
  for i=1,#self do
    t[i] = self[i]
  end
  return t
end

-- PointXYZ metatype
local PointXYZ = {
  prototype = {
    totensor = totensor,
    set = set,
    hasNormal = false
  },
  __len = len,
  __eq = eq,
  fields = { 'x', 'y', 'z', 'w' }
}

PointXYZ.__pairs = createpairs(PointXYZ.fields)
function PointXYZ:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZ.prototype[i] end end
function PointXYZ:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointXYZ:__tostring() return string.format('{ x:%f, y:%f, z:%f }', self.x, self.y, self.z) end 
ffi.metatype(pcl.PointXYZ, PointXYZ)
pcl.metatype[pcl.PointXYZ] = PointXYZ

-- PointXYZI metatype
local PointXYZI = {
  prototype = {
    totensor = totensor,
    set = set,
    hasNormal = false
  },
  __len = len,
  __eq = eq,
  fields = { 'x', 'y', 'z', 'w', 'i', '_1', '_2', '_3' }
}

PointXYZI.__pairs = createpairs(PointXYZI.fields)
function PointXYZI:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZI.prototype[i] end end
function PointXYZI:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointXYZI:__tostring() return string.format('{ x:%f, y:%f, z:%f, intensity:%f }', self.x, self.y, self.z, self.intensity) end 
ffi.metatype(pcl.PointXYZI, PointXYZI)
pcl.metatype[pcl.PointXYZI] = PointXYZI

-- PointXYZRGABmetatype
local PointXYZRGBA = {
  prototype = {
    tensor = totensor,
    set = set,
    hasNormal = false
  },
  __eq = eq,
  __len = len,
  fields = { 'x', 'y', 'z', 'w', 'rgba', '_1', '_2', '_3' }
}

PointXYZRGBA.__pairs = createpairs(PointXYZRGBA.fields)
function PointXYZRGBA:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZRGBA.prototype[i] end end
function PointXYZRGBA:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointXYZRGBA:__tostring() return string.format('{ x:%f, y:%f, z:%f, rgba: %08X }', self.x, self.y, self.z, self.rgba) end 
ffi.metatype(pcl.PointXYZRGBA, PointXYZRGBA)
pcl.metatype[pcl.PointXYZRGBA] = PointXYZRGBA

-- Normal metatype
local Normal = {
  prototype = {
    tensor = totensor,
    set = set,
    hasNormal = true
  },
  __eq = eq,
  __len = len,
  fields = { 'normal_x', 'normal_y', 'normal_z', '_1', 'curvature', '_2', '_3', '_3' }
}
Normal.__pairs = createpairs(Normal.fields)
function Normal:__index(i) if type(i) == "number" then return self.data[i-1] else return Normal.prototype[i] end end
function Normal:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function Normal:__tostring()
  return string.format('{ normal_x:%f, normal_y:%f, normal_z:%f, curvature:%f }', 
    self.normal_x, self.normal_y, self.normal_z, self.curvature) 
end 
ffi.metatype(pcl.Normal, Normal)
pcl.metatype[pcl.Normal] = Normal

-- PointNormal metatype
local PointNormal = {
  prototype = {
    tensor = totensor,
    set = set,
    hasNormal = true
  },
  __eq = eq,
  __len = len,
  fields = { 'x', 'y', 'z', 'w', 'normal_x', 'normal_y', 'normal_z', '_1', 'curvature', '_2', '_3', '_4' }
}

PointNormal.__pairs = createpairs(PointNormal.fields)
function PointNormal:__index(i) if type(i) == "number" then return self.data[i-1] else return PointNormal.prototype[i] end end
function PointNormal:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointNormal:__tostring()
  return string.format('{ x:%f, y:%f, z:%f, normal_x:%f, normal_y:%f, normal_z:%f, curvature:%f }', 
    self.x, self.y, self.z, self.normal_x, self.normal_y, self.normal_z, self.curvature) 
end 
ffi.metatype(pcl.PointNormal, PointNormal)
pcl.metatype[pcl.PointNormal] = PointNormal

-- PointXYZINormal metatype
local PointXYZINormal = {
  prototype = {
    tensor = totensor,
    set = set,
    hasNormal = true
  },
  __eq = eq,
  __len = len,
  fields = { 'x', 'y', 'z', 'w', 'normal_x', 'normal_y', 'normal_z', '_1', 'intensity', 'curvature', '_2', '_3' } 
}
PointXYZINormal.__pairs = createpairs(PointXYZINormal.fields)
function PointXYZINormal:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZINormal.prototype[i] end end
function PointXYZINormal:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointXYZINormal:__tostring()
  return string.format('{ x:%f, y:%f, z:%f, normal_x:%f, normal_y:%f, normal_z:%f, intensity:%f, curvature:%f }', 
    self.x, self.y, self.z, self.normal_x, self.normal_y, self.normal_z, self.intensity, self.curvature) 
end 
ffi.metatype(pcl.PointXYZINormal, PointXYZINormal)
pcl.metatype[pcl.PointXYZINormal] = PointXYZINormal

-- PointXYZRGBANormal metatype
local PointXYZRGBANormal = {
  prototype = {
    tensor = totensor,
    set = set,
    hasNormal = true
  },
  __eq = eq,
  __len = len,
  fields = { 'x', 'y', 'z', 'w', 'normal_x', 'normal_y', 'normal_z', '_1', 'rgba', 'curvature', '_2', '_3' }
}
PointXYZRGBANormal.__pairs = createpairs(PointXYZRGBANormal.fields)
function PointXYZRGBANormal:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZRGBANormal.prototype[i] end end
function PointXYZRGBANormal:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointXYZRGBANormal:__tostring()
  return string.format('{ x:%f, y:%f, z:%f, normal_x:%f, normal_y:%f, normal_z:%f, rgba:%08X, curvature:%f }', 
    self.x, self.y, self.z, self.normal_x, self.normal_y, self.normal_z, self.rgba, self.curvature) 
end 
ffi.metatype(pcl.PointXYZRGBANormal, PointXYZRGBANormal)
pcl.metatype[pcl.PointXYZRGBANormal] = PointXYZRGBANormal

pcl.range = {
  double = {
    min = -1.7976931348623157E+308, 
    max =  1.7976931348623157E+308,
    eps =  2.22507385850720138e-308
  },
  float = {
    min = -3.402823e+38,
    max =  3.402823e+38,
    eps =  1.175494351e-38
  },
  int16 = {
    min = -32768,
    max =  32767
  },
  uint16 = {
    min =  0,
    max =  0xffff
  },
  int32 = {
    min = −2147483648,
    max =  2147483647
  },
  uint32 = {
    min =  0,
    max =  0xffffffff
  },
  int64 = {
    min = −9223372036854775808,
    max =  9223372036854775807
  }, 
  uint64 = {
    min =  0,
    max =  0xffffffffffffffff
  }
}

return pcl
