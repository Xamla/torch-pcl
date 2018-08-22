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
typedef struct PointXYZRGBNormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { "..PCL_RGB.." float curvature; }; float data_c[4]; }; } PointXYZRGBNormal; \z
typedef struct PointXYZINormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float intensity; float curvature; }; float data_c[4]; }; } PointXYZINormal; \z
typedef struct Boundary { uint8_t boundary_point; } Boundary;\z
typedef struct Label { uint32_t label; } Label;\z
" ..
[[
typedef struct FPFHSignature33 { float histogram[33]; } FPFHSignature33;
typedef struct VFHSignature308 { float histogram[308]; } VFHSignature308;
typedef struct Correspondence { int index_query; int index_match; float distance; } Correspondence;
typedef struct _PointsBuffer { THFloatStorage* storage; uint32_t width, height, dim; } _PointsBuffer;
typedef struct OpenNI2CameraParameters { double focal_length_x; double focal_length_y; double principal_point_x; double principal_point_y; } OpenNI2CameraParameters;
typedef struct PointCloud_XYZ {} PointCloud_XYZ;
typedef struct PointCloud_XYZI {} PointCloud_XYZI;
typedef struct PointCloud_XYZRGBA {} PointCloud_XYZRGBA;
typedef struct PointCloud_XYZNormal {} PointCloud_XYZNormal;
typedef struct PointCloud_XYZINormal {} PointCloud_XYZINormal;
typedef struct PointCloud_XYZRGBNormal {} PointCloud_XYZRGBNormal;
typedef struct PointCloud_Normal {} PointCloud_Normal;
typedef struct PointCloud_FPFHSignature33 {} PointCloud_FPFHSignature33;
typedef struct PointCloud_VFHSignature308 {} PointCloud_VFHSignature308;
typedef struct PointCloud_Boundary {} PointCloud_Boundary;
typedef struct PointCloud_Label {} PointCloud_Label;

void* pcl_CloudViewer_new(const char *window_name);
void pcl_CloudViewer_delete(void *self);
bool pcl_CloudViewer_wasStopped(void *self, int millis_to_wait);

void pcl_Primitive_XYZ_createSphere(PointCloud_XYZ *output, double radius, double thetaRes, double phiRes, int samples, float resolution);
void pcl_Primitive_XYZ_createCube(PointCloud_XYZ *output, double x, double y, double z, int samples, float resolution);
void pcl_Primitive_XYZ_createCylinder(PointCloud_XYZ *output, double height, double radius, int facets, int samples, float resolution);
void pcl_Primitive_XYZ_createCone(PointCloud_XYZ *output, double height, double radius, int facets, int samples, float resolution);
void pcl_Primitive_XYZ_createPlatonicSolid(PointCloud_XYZ *output, int solidType, int samples, float resolution);
void pcl_Primitive_XYZ_createPlane(PointCloud_XYZ *output, double x1, double y1, double z1, double x2, double y2, double z2, int samples, float resolution);
void pcl_Primitive_XYZ_createDisk(PointCloud_XYZ *output, double innerRadius, double outerRadius, int radialResolution, int samples, float resolution);

typedef struct Indices {} Indices;
Indices *pcl_Indices_new();
Indices *pcl_Indices_clone(Indices *self);
void pcl_Indices_delete(Indices *self);
Indices *pcl_Indices_fromPtr(Indices *existing);
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

typedef struct IndicesVector {} IndicesVector;
IndicesVector *pcl_IndicesVector_new();
void pcl_IndicesVector_delete(IndicesVector *self);
unsigned int pcl_IndicesVector_size(IndicesVector *self);
void pcl_IndicesVector_getAt(IndicesVector *self, size_t pos, Indices *result);
void pcl_IndicesVector_setAt(IndicesVector *self, size_t pos, Indices *value);
void pcl_IndicesVector_push_back(IndicesVector *self, Indices *value);
void pcl_IndicesVector_pop_back(IndicesVector *self);
void pcl_IndicesVector_clear(IndicesVector *self);
void pcl_IndicesVector_insert(IndicesVector *self, size_t pos, Indices *value);
void pcl_IndicesVector_erase(IndicesVector *self, size_t begin, size_t end);
bool pcl_IndicesVector_empty(IndicesVector *self);

typedef struct Correspondences {} Correspondences;
Correspondences *pcl_Correspondences_new();
Correspondences *pcl_Correspondences_clone(Correspondences *self);
void pcl_Correspondences_delete(Correspondences *self);
int pcl_Correspondences_size(Correspondences *self);
Correspondence pcl_Correspondences_getAt(Correspondences *self, size_t pos);
void pcl_Correspondences_setAt(Correspondences *self, size_t pos, const Correspondence &value);
void pcl_Correspondences_push_back(Correspondences *self, const Correspondence &value);
void pcl_Correspondences_pop_back(Correspondences *self);
void pcl_Correspondences_clear(Correspondences *self);
void pcl_Correspondences_insert(Correspondences *self, size_t pos, const Correspondence &value);
void pcl_Correspondences_erase(Correspondences *self, size_t begin, size_t end);
bool pcl_Correspondences_empty(Correspondences *self);

struct NarfKeypointParameters
{
  float support_size;
  int max_no_of_interest_points;
  float min_distance_between_interest_points;
  float optimal_distance_to_high_surface_change;
  float min_interest_value;
  float min_surface_change_score;
  int optimal_range_image_patch_size;
  float distance_for_additional_points;
  bool add_points_on_straight_edges;
  bool do_non_maximum_suppression;
  bool no_of_polynomial_approximations_per_point;
  int max_no_of_threads;
  bool use_recursive_scale_reduction;
  bool calculate_sparse_interest_image;
};
]]
ffi.cdef(cdef)

-- check whether torch-ros has been loaded before, if not declare pcl_PCLPointCloud2 structure.
if package.loaded['ros'] == nil then
  ffi.cdef('typedef struct pcl_PCLPointCloud2 {} pcl_PCLPointCloud2;')
end

local pcl_PointCloud_declaration = [[
PointCloud_TYPE_KEY* pcl_PointCloud_TYPE_KEY_new(uint32_t width, uint32_t height);
PointCloud_TYPE_KEY* pcl_PointCloud_TYPE_KEY_clone(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_delete(PointCloud_TYPE_KEY *self);
uint32_t pcl_PointCloud_TYPE_KEY_getHeaderSeq(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_setHeaderSeq(PointCloud_TYPE_KEY *self, uint32_t value);
int32_t pcl_PointCloud_TYPE_KEY_getHeaderStamp_sec(PointCloud_TYPE_KEY *self);
int32_t pcl_PointCloud_TYPE_KEY_getHeaderStamp_nsec(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_setHeaderStamp(PointCloud_TYPE_KEY *self, int32_t sec, int32_t nsec);
const char *pcl_PointCloud_TYPE_KEY_getHeaderFrameId(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_setHeaderFrameId(PointCloud_TYPE_KEY *self, const char *value);
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
void pcl_PointCloud_TYPE_KEY_fromPCLPointCloud2(void *cloud, pcl_PCLPointCloud2 *msg);
void pcl_PointCloud_TYPE_KEY_toPCLPointCloud2(void *cloud, pcl_PCLPointCloud2 *msg);
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
void pcl_PointCloud_TYPE_KEY_copyXYZRGBNormal(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_XYZRGBNormal *cloud_out);
void pcl_PointCloud_TYPE_KEY_copyNormal(PointCloud_TYPE_KEY *cloud_in, Indices *indices, PointCloud_Normal *cloud_out);
]]

local KdTreeFLANN_declarations = [[
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
void pcl_Filter_TYPE_KEY_passThrough_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, const char* fieldName, float min, float max, bool negative, Indices *removed_indices, bool keep_organized);
void pcl_Filter_TYPE_KEY_passThrough_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, const char* fieldName, float min, float max, bool negative, Indices *removed_indices, bool keep_organized);
void pcl_Filter_TYPE_KEY_cropBox_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, THFloatTensor *transform, bool negative, Indices *removed_indices, bool keep_organized);
void pcl_Filter_TYPE_KEY_cropBox_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, THFloatTensor *min, THFloatTensor *max, THFloatTensor *rotation, THFloatTensor *translation, THFloatTensor *transform, bool negative, Indices *removed_indices, bool keep_organized);
void pcl_Filter_TYPE_KEY_cropSphere_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_cropSphere_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, THFloatTensor *center, double radius, THFloatTensor *transform, bool negative, Indices *removed_indices);
void pcl_Filter_TYPE_KEY_voxelGrid(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, float lx, float ly, float lz);
void pcl_Filter_TYPE_KEY_statisticalOutlierRemoval_Indices(PointCloud_TYPE_KEY *input, Indices *indices, Indices *output, int meanK, double stddevMulThresh, bool negative, Indices *removed_indices, bool keep_organized);
void pcl_Filter_TYPE_KEY_statisticalOutlierRemoval_Cloud(PointCloud_TYPE_KEY *input, Indices *indices, PointCloud_TYPE_KEY *output, int meanK, double stddevMulThresh, bool negative, Indices *removed_indices, bool keep_organized);
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

typedef struct {} OpenNI2Stream_TYPE_KEY;
OpenNI2Stream_TYPE_KEY* pcl_OpenNI2Stream_TYPE_KEY_new(const char* device_id, int max_backlog, bool grab_RGB, bool grab_Depth, bool grab_IR);
void pcl_OpenNI2Stream_TYPE_KEY_delete(OpenNI2Stream_TYPE_KEY *self);
void pcl_OpenNI2Stream_TYPE_KEY_start(OpenNI2Stream_TYPE_KEY *self);
void pcl_OpenNI2Stream_TYPE_KEY_stop(OpenNI2Stream_TYPE_KEY *self);
void* pcl_OpenNI2Stream_TYPE_KEY_read(OpenNI2Stream_TYPE_KEY *self, int timeout_milliseconds);
void pcl_OpenNI2Stream_TYPE_KEY_readRGBImage(OpenNI2Stream_TYPE_KEY *self, int timeout_milliseconds, THByteTensor *output);
void pcl_OpenNI2Stream_TYPE_KEY_readDepthImage(OpenNI2Stream_TYPE_KEY *self, int timeout_milliseconds, THShortTensor *output);
void pcl_OpenNI2Stream_TYPE_KEY_readIRImage(OpenNI2Stream_TYPE_KEY *self, int timeout_milliseconds, THShortTensor *output);
void pcl_OpenNI2Stream_TYPE_KEY_getRGBCameraIntrinsics(OpenNI2Stream_TYPE_KEY *self, OpenNI2CameraParameters& p);
void pcl_OpenNI2Stream_TYPE_KEY_setRGBCameraIntrinsics(OpenNI2Stream_TYPE_KEY *self, const OpenNI2CameraParameters& p);
void pcl_OpenNI2Stream_TYPE_KEY_getDepthCameraIntrinsics(OpenNI2Stream_TYPE_KEY *self, OpenNI2CameraParameters& p);
void pcl_OpenNI2Stream_TYPE_KEY_setDepthCameraIntrinsics(OpenNI2Stream_TYPE_KEY *self, const OpenNI2CameraParameters& p);
const char* pcl_OpenNI2Stream_TYPE_KEY_getName(OpenNI2Stream_TYPE_KEY *self);
float pcl_OpenNI2Stream_TYPE_KEY_getFramesPerSecond(OpenNI2Stream_TYPE_KEY *self);

typedef struct {} SIFTKeypoint_TYPE_KEY;
SIFTKeypoint_TYPE_KEY* pcl_SIFTKeypoint_TYPE_KEY_new();
void pcl_SIFTKeypoint_TYPE_KEY_delete(SIFTKeypoint_TYPE_KEY *self);
void pcl_SIFTKeypoint_TYPE_KEY_setInputCloud(SIFTKeypoint_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_SIFTKeypoint_TYPE_KEY_setSearchMethod_Octree(SIFTKeypoint_TYPE_KEY *self, OctreePointCloudSearch_TYPE_KEY *octree);
void pcl_SIFTKeypoint_TYPE_KEY_setSearchMethod_KdTree(SIFTKeypoint_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *kdtree);
void pcl_SIFTKeypoint_TYPE_KEY_setScales(SIFTKeypoint_TYPE_KEY *self, float min_scale, int nr_octaves, int nr_scales_per_octave);
void pcl_SIFTKeypoint_TYPE_KEY_setMinimumContrast(SIFTKeypoint_TYPE_KEY *self, float min_contrast);
void pcl_SIFTKeypoint_TYPE_KEY_compute(SIFTKeypoint_TYPE_KEY *self, PointCloud_XYZ *output);

typedef struct {} FPFHEstimation_TYPE_KEY;
FPFHEstimation_TYPE_KEY* pcl_FPFHEstimation_TYPE_KEY_new();
void pcl_FPFHEstimation_TYPE_KEY_delete(FPFHEstimation_TYPE_KEY *self);
void pcl_FPFHEstimation_TYPE_KEY_setInputCloud(FPFHEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_FPFHEstimation_TYPE_KEY_setInputNormals(FPFHEstimation_TYPE_KEY *self, PointCloud_Normal *normals);
void pcl_FPFHEstimation_TYPE_KEY_setIndices(FPFHEstimation_TYPE_KEY *self, Indices *indices);
void pcl_FPFHEstimation_TYPE_KEY_compute(FPFHEstimation_TYPE_KEY *self, PointCloud_FPFHSignature33 *output);
void pcl_FPFHEstimation_TYPE_KEY_setKSearch(FPFHEstimation_TYPE_KEY *self, int k);
int pcl_FPFHEstimation_TYPE_KEY_getKSearch(FPFHEstimation_TYPE_KEY *self);
void pcl_FPFHEstimation_TYPE_KEY_setRadiusSearch(FPFHEstimation_TYPE_KEY *self, double radius);
double pcl_FPFHEstimation_TYPE_KEY_getRadiusSearch(FPFHEstimation_TYPE_KEY *self);

typedef struct {} VFHEstimation_TYPE_KEY;
VFHEstimation_TYPE_KEY* pcl_VFHEstimation_TYPE_KEY_new();
void pcl_VFHEstimation_TYPE_KEY_delete(VFHEstimation_TYPE_KEY *self);
void pcl_VFHEstimation_TYPE_KEY_setInputCloud(VFHEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_VFHEstimation_TYPE_KEY_setInputNormals(VFHEstimation_TYPE_KEY *self, PointCloud_Normal *normals);
void pcl_VFHEstimation_TYPE_KEY_setIndices(VFHEstimation_TYPE_KEY *self, Indices *indices);
void pcl_VFHEstimation_TYPE_KEY_getViewPoint(VFHEstimation_TYPE_KEY *self, THFloatTensor* out_pt);
void pcl_VFHEstimation_TYPE_KEY_setViewPoint(VFHEstimation_TYPE_KEY *self, THFloatTensor *pt);
void pcl_VFHEstimation_TYPE_KEY_setUseGivenNormal(VFHEstimation_TYPE_KEY *self, bool use);
void pcl_VFHEstimation_TYPE_KEY_setNormalToUse(VFHEstimation_TYPE_KEY *self, THFloatTensor *normal);
void pcl_VFHEstimation_TYPE_KEY_setUseGivenCentroid(VFHEstimation_TYPE_KEY *self, bool use);
void pcl_VFHEstimation_TYPE_KEY_setCentroidToUse(VFHEstimation_TYPE_KEY *self, THFloatTensor *centroid);
void pcl_VFHEstimation_TYPE_KEY_setNormalizeBins(VFHEstimation_TYPE_KEY *self, bool normalize);
void pcl_VFHEstimation_TYPE_KEY_setNormalizeDistance(VFHEstimation_TYPE_KEY *self, bool normalize);
void pcl_VFHEstimation_TYPE_KEY_setFillSizeComponent(VFHEstimation_TYPE_KEY *self, bool fill_size);
void pcl_VFHEstimation_TYPE_KEY_compute(VFHEstimation_TYPE_KEY *self, PointCloud_VFHSignature308 *output);
void pcl_VFHEstimation_TYPE_KEY_setKSearch(VFHEstimation_TYPE_KEY *self, int k);
int pcl_VFHEstimation_TYPE_KEY_getKSearch(VFHEstimation_TYPE_KEY *self);
void pcl_VFHEstimation_TYPE_KEY_setRadiusSearch(VFHEstimation_TYPE_KEY *self, double radius);
double pcl_VFHEstimation_TYPE_KEY_getRadiusSearch(VFHEstimation_TYPE_KEY *self);
]]

local pcl_CorrespondenceEstimation_declaration = [[
typedef struct {} CorrespondenceEstimation_TYPE_KEY;
CorrespondenceEstimation_TYPE_KEY *pcl_CorrespondenceEstimation_TYPE_KEY_new();
void pcl_CorrespondenceEstimation_TYPE_KEY_delete(CorrespondenceEstimation_TYPE_KEY *self);
void pcl_CorrespondenceEstimation_TYPE_KEY_setInputSource(CorrespondenceEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_CorrespondenceEstimation_TYPE_KEY_setInputTarget(CorrespondenceEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_CorrespondenceEstimation_TYPE_KEY_setIndicesSource(CorrespondenceEstimation_TYPE_KEY *self, Indices *indices);
void pcl_CorrespondenceEstimation_TYPE_KEY_setIndicesTarget(CorrespondenceEstimation_TYPE_KEY *self, Indices *indices);
void pcl_CorrespondenceEstimation_TYPE_KEY_setSearchMethodSource(CorrespondenceEstimation_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *kdtree, bool force_no_recompute);
void pcl_CorrespondenceEstimation_TYPE_KEY_setSearchMethodTarget(CorrespondenceEstimation_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *kdtree, bool force_no_recompute);
void pcl_CorrespondenceEstimation_TYPE_KEY_determineCorrespondences(CorrespondenceEstimation_TYPE_KEY *self, Correspondences *correspondences, double max_distance);
void pcl_CorrespondenceEstimation_TYPE_KEY_determineReciprocalCorrespondences(CorrespondenceEstimation_TYPE_KEY *self, Correspondences *correspondences, double max_distance);
]]

local pcl_SampleConsensusPrerejective_declaration = [[
typedef struct {} SampleConsensusPrerejective_TYPE_KEY;
SampleConsensusPrerejective_TYPE_KEY *pcl_SampleConsensusPrerejective_TYPE_KEY_new();
void pcl_SampleConsensusPrerejective_TYPE_KEY_delete(SampleConsensusPrerejective_TYPE_KEY *self);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setInputSource(SampleConsensusPrerejective_TYPE_KEY *self, PointCloud_TYPE_KEY *source_cloud);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setSourceFeatures(SampleConsensusPrerejective_TYPE_KEY *self, PointCloud_FPFHSignature33* source_features);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setInputTarget(SampleConsensusPrerejective_TYPE_KEY *self, PointCloud_TYPE_KEY *target_cloud);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setTargetFeatures(SampleConsensusPrerejective_TYPE_KEY *self, PointCloud_FPFHSignature33* target_features);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setMaximumIterations(SampleConsensusPrerejective_TYPE_KEY *self, int max_iterations);
int pcl_SampleConsensusPrerejective_TYPE_KEY_getMaximumIterations(SampleConsensusPrerejective_TYPE_KEY *self);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setNumberOfSamples(SampleConsensusPrerejective_TYPE_KEY *self, int nr_samples);
int pcl_SampleConsensusPrerejective_TYPE_KEY_getNumberOfSamples(SampleConsensusPrerejective_TYPE_KEY *self);
double pcl_SampleConsensusPrerejective_TYPE_KEY_getMaxCorrespondenceDistance(SampleConsensusPrerejective_TYPE_KEY *self);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setMaxCorrespondenceDistance(SampleConsensusPrerejective_TYPE_KEY *self, double distance);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setCorrespondenceRandomness(SampleConsensusPrerejective_TYPE_KEY *self, int k);
int pcl_SampleConsensusPrerejective_TYPE_KEY_getCorrespondenceRandomness(SampleConsensusPrerejective_TYPE_KEY *self);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setSimilarityThreshold(SampleConsensusPrerejective_TYPE_KEY *self, float similarity_threshold);
float pcl_SampleConsensusPrerejective_TYPE_KEY_getSimilarityThreshold(SampleConsensusPrerejective_TYPE_KEY *self);
void pcl_SampleConsensusPrerejective_TYPE_KEY_setInlierFraction(SampleConsensusPrerejective_TYPE_KEY *self, float inlier_fraction);
float pcl_SampleConsensusPrerejective_TYPE_KEY_getInlierFraction(SampleConsensusPrerejective_TYPE_KEY *self);
void pcl_SampleConsensusPrerejective_TYPE_KEY_getInliers(SampleConsensusPrerejective_TYPE_KEY *self, Indices *indices);
void pcl_SampleConsensusPrerejective_TYPE_KEY_getFinalTransformation(SampleConsensusPrerejective_TYPE_KEY *self, THFloatTensor *output);
double pcl_SampleConsensusPrerejective_TYPE_KEY_getFitnessScore(SampleConsensusPrerejective_TYPE_KEY *self, double max_range);
void pcl_SampleConsensusPrerejective_TYPE_KEY_align(SampleConsensusPrerejective_TYPE_KEY *self, PointCloud_TYPE_KEY *output, THFloatTensor *guess);
]]

local pcl_BoundaryEstimation_declaration = [[
typedef struct {} BoundaryEstimation_TYPE_KEY;
BoundaryEstimation_TYPE_KEY* pcl_BoundaryEstimation_TYPE_KEY_new();
void pcl_BoundaryEstimation_TYPE_KEY_delete(BoundaryEstimation_TYPE_KEY *self);
void pcl_BoundaryEstimation_TYPE_KEY_setInputCloud(BoundaryEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_BoundaryEstimation_TYPE_KEY_setInputNormals(BoundaryEstimation_TYPE_KEY *self, PointCloud_Normal *normals);
void pcl_BoundaryEstimation_TYPE_KEY_setIndices(BoundaryEstimation_TYPE_KEY *self, Indices *indices);
void pcl_BoundaryEstimation_TYPE_KEY_setAngleThreshold(BoundaryEstimation_TYPE_KEY *self, float angle);
float pcl_BoundaryEstimation_TYPE_KEY_getAngleThreshold(BoundaryEstimation_TYPE_KEY *self);
void pcl_BoundaryEstimation_TYPE_KEY_setSearchMethod_Octree(BoundaryEstimation_TYPE_KEY *self, OctreePointCloudSearch_TYPE_KEY *octree);
void pcl_BoundaryEstimation_TYPE_KEY_setSearchMethod_KdTree(BoundaryEstimation_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *kdtree);
void pcl_BoundaryEstimation_TYPE_KEY_setKSearch(BoundaryEstimation_TYPE_KEY *self, int k);
int pcl_BoundaryEstimation_TYPE_KEY_getKSearch(BoundaryEstimation_TYPE_KEY *self);
void pcl_BoundaryEstimation_TYPE_KEY_setRadiusSearch(BoundaryEstimation_TYPE_KEY *self, double radius);
double pcl_BoundaryEstimation_TYPE_KEY_getRadiusSearch(BoundaryEstimation_TYPE_KEY *self);
void pcl_BoundaryEstimation_TYPE_KEY_compute(BoundaryEstimation_TYPE_KEY *self, PointCloud_Boundary *output);
void pcl_BoundaryEstimation_TYPE_KEY_computeIndices(BoundaryEstimation_TYPE_KEY *self, Indices *indices);
]]

local pcl_SACSegmentation_declaration = [[
typedef struct {} SACSegmentationHandle_TYPE_KEY;
typedef struct {} SACSegmentation_TYPE_KEY;
SACSegmentationHandle_TYPE_KEY *pcl_SACSegmentation_TYPE_KEY_new();
void   pcl_SACSegmentation_TYPE_KEY_delete(SACSegmentationHandle_TYPE_KEY *handle);
SACSegmentation_TYPE_KEY *pcl_SACSegmentation_TYPE_KEY_SACSegmentation_ptr(SACSegmentationHandle_TYPE_KEY *handle);
void   pcl_SACSegmentation_TYPE_KEY_setInputCloud(SACSegmentation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void   pcl_SACSegmentation_TYPE_KEY_setIndices(SACSegmentation_TYPE_KEY *self, Indices *indices);
void   pcl_SACSegmentation_TYPE_KEY_setMethodType(SACSegmentation_TYPE_KEY *self, int method);
int    pcl_SACSegmentation_TYPE_KEY_getMethodType(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_setModelType(SACSegmentation_TYPE_KEY *self, int model);
int    pcl_SACSegmentation_TYPE_KEY_getModelType(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_setDistanceThreshold(SACSegmentation_TYPE_KEY *self, double threshold);
double pcl_SACSegmentation_TYPE_KEY_getDistanceThreshold(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_setMaxIterations(SACSegmentation_TYPE_KEY *self, int max_iterations);
int    pcl_SACSegmentation_TYPE_KEY_getMaxIterations(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_setProbability(SACSegmentation_TYPE_KEY *self, double probability);
double pcl_SACSegmentation_TYPE_KEY_getProbability(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_setOptimizeCoefficients(SACSegmentation_TYPE_KEY *self, bool optimize);
bool   pcl_SACSegmentation_TYPE_KEY_getOptimizeCoefficients(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_setSamplesMaxDist_KdTree(SACSegmentation_TYPE_KEY *self, double radius, KdTreeFLANN_TYPE_KEY *search);
void   pcl_SACSegmentation_TYPE_KEY_setSamplesMaxDist_Octree(SACSegmentation_TYPE_KEY *self, double radius, OctreePointCloudSearch_TYPE_KEY *search);
void   pcl_SACSegmentation_TYPE_KEY_setRadiusLimits(SACSegmentation_TYPE_KEY *self, double min_radius, double max_radius);
void   pcl_SACSegmentation_TYPE_KEY_setAxis(SACSegmentation_TYPE_KEY *self, THFloatTensor *axis);
void   pcl_SACSegmentation_TYPE_KEY_getAxis(SACSegmentation_TYPE_KEY *self, THFloatTensor *result);
void   pcl_SACSegmentation_TYPE_KEY_setEpsAngle(SACSegmentation_TYPE_KEY *self, double ea);
double pcl_SACSegmentation_TYPE_KEY_getEpsAngle(SACSegmentation_TYPE_KEY *self);
void   pcl_SACSegmentation_TYPE_KEY_segment(SACSegmentation_TYPE_KEY *self, Indices *inliers, THFloatTensor *model_coefficients);

typedef struct {} SACSegmentationFromNormalsHandle_TYPE_KEY;
typedef struct {} SACSegmentationFromNormals_TYPE_KEY;
SACSegmentationFromNormalsHandle_TYPE_KEY *pcl_SACSegmentationFromNormals_TYPE_KEY_new();
void pcl_SACSegmentationFromNormals_TYPE_KEY_delete(SACSegmentationFromNormalsHandle_TYPE_KEY *handle);
SACSegmentationFromNormals_TYPE_KEY *pcl_SACSegmentationFromNormals_TYPE_KEY_SACSegmentationFromNormals_ptr(SACSegmentationFromNormalsHandle_TYPE_KEY *handle);
SACSegmentation_TYPE_KEY *pcl_SACSegmentationFromNormals_TYPE_KEY_SACSegmentation_ptr(SACSegmentationFromNormalsHandle_TYPE_KEY *handle);
void pcl_SACSegmentationFromNormals_TYPE_KEY_setInputNormals(SACSegmentationFromNormals_TYPE_KEY *self, PointCloud_Normal *normals);
void pcl_SACSegmentationFromNormals_TYPE_KEY_setNormalDistanceWeight(SACSegmentationFromNormals_TYPE_KEY *self, double distance_weight);
void pcl_SACSegmentationFromNormals_TYPE_KEY_setMinMaxOpeningAngle(SACSegmentationFromNormals_TYPE_KEY *self, double min_angle, double max_angle);
void pcl_SACSegmentationFromNormals_TYPE_KEY_setDistanceFromOrigin(SACSegmentationFromNormals_TYPE_KEY *self, double d);
]]

local pcl_EuclideanClusterExtraction_declaration = [[
typedef struct {} EuclideanClusterExtractionHandle_TYPE_KEY;
typedef struct {} EuclideanClusterExtraction_TYPE_KEY;

EuclideanClusterExtractionHandle_TYPE_KEY *pcl_EuclideanClusterExtraction_TYPE_KEY_new();
void pcl_EuclideanClusterExtraction_TYPE_KEY_delete(EuclideanClusterExtractionHandle_TYPE_KEY *handle);
EuclideanClusterExtraction_TYPE_KEY *pcl_EuclideanClusterExtraction_TYPE_KEY_EuclideanClusterExtraction_ptr(EuclideanClusterExtractionHandle_TYPE_KEY *handle);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setInputCloud(EuclideanClusterExtraction_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setIndices(EuclideanClusterExtraction_TYPE_KEY *self, Indices *indices);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setSearchMethod_Octree(EuclideanClusterExtraction_TYPE_KEY *self, OctreePointCloudSearch_TYPE_KEY *octree);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setSearchMethod_KdTree(EuclideanClusterExtraction_TYPE_KEY *self, KdTreeFLANN_TYPE_KEY *kdtree);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setClusterTolerance(EuclideanClusterExtraction_TYPE_KEY *self, double tolerance);
double pcl_EuclideanClusterExtraction_TYPE_KEY_getClusterTolerance(EuclideanClusterExtraction_TYPE_KEY *self);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setMinClusterSize(EuclideanClusterExtraction_TYPE_KEY *self, int min_cluster_size);
int pcl_EuclideanClusterExtraction_TYPE_KEY_getMinClusterSize(EuclideanClusterExtraction_TYPE_KEY *self);
void pcl_EuclideanClusterExtraction_TYPE_KEY_setMaxClusterSize(EuclideanClusterExtraction_TYPE_KEY *self, int max_cluster_size);
int pcl_EuclideanClusterExtraction_TYPE_KEY_getMaxClusterSize(EuclideanClusterExtraction_TYPE_KEY *self);
void pcl_EuclideanClusterExtraction_TYPE_KEY_extract(EuclideanClusterExtraction_TYPE_KEY *self, IndicesVector *clusters);
]]

local pcl_OrganizedEdge_declaration = [[
typedef struct {} OrganizedEdgeBaseHandle_TYPE_KEY;
typedef struct {} OrganizedEdgeBase_TYPE_KEY;
OrganizedEdgeBaseHandle_TYPE_KEY *pcl_OrganizedEdgeBase_TYPE_KEY_new();
void pcl_OrganizedEdgeBase_TYPE_KEY_delete(OrganizedEdgeBaseHandle_TYPE_KEY *handle);
OrganizedEdgeBase_TYPE_KEY *pcl_OrganizedEdgeBase_TYPE_KEY_OrganizedEdgeBase_ptr(OrganizedEdgeBaseHandle_TYPE_KEY *handle);
void pcl_OrganizedEdgeBase_TYPE_KEY_setInputCloud(OrganizedEdgeBase_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_OrganizedEdgeBase_TYPE_KEY_setDepthDisconThreshold(OrganizedEdgeBase_TYPE_KEY *self, float th);
void pcl_OrganizedEdgeBase_TYPE_KEY_setMaxSearchNeighbors(OrganizedEdgeBase_TYPE_KEY *self, int max_dist);
void pcl_OrganizedEdgeBase_TYPE_KEY_setEdgeType(OrganizedEdgeBase_TYPE_KEY *self, int edge_types);
void pcl_OrganizedEdgeBase_TYPE_KEY_compute(OrganizedEdgeBase_TYPE_KEY *self, PointCloud_Label *labels, IndicesVector *label_indices);

typedef struct {} OrganizedEdgeFromNormalsHandle_TYPE_KEY;
typedef struct {} OrganizedEdgeFromNormals_TYPE_KEY;
OrganizedEdgeFromNormalsHandle_TYPE_KEY *pcl_OrganizedEdgeFromNormals_TYPE_KEY_new();
void pcl_OrganizedEdgeFromNormals_TYPE_KEY_delete(OrganizedEdgeFromNormalsHandle_TYPE_KEY *handle);
OrganizedEdgeBase_TYPE_KEY *pcl_OrganizedEdgeFromNormals_TYPE_KEY_OrganizedEdgeBase_ptr(OrganizedEdgeFromNormalsHandle_TYPE_KEY *handle);
OrganizedEdgeFromNormals_TYPE_KEY *pcl_OrganizedEdgeFromNormals_TYPE_KEY_OrganizedEdgeFromNormals_ptr(OrganizedEdgeFromNormalsHandle_TYPE_KEY *handle);
void pcl_OrganizedEdgeFromNormals_TYPE_KEY_setInputNormals(OrganizedEdgeFromNormals_TYPE_KEY *self, PointCloud_Normal *normals);
void pcl_OrganizedEdgeFromNormals_TYPE_KEY_setHCCannyLowThreshold(OrganizedEdgeFromNormals_TYPE_KEY *self, float th);
void pcl_OrganizedEdgeFromNormals_TYPE_KEY_setHCCannyHighThreshold(OrganizedEdgeFromNormals_TYPE_KEY *self, float th);
void pcl_OrganizedEdgeFromNormals_TYPE_KEY_compute(OrganizedEdgeFromNormals_TYPE_KEY *self, PointCloud_Label *labels, IndicesVector *label_indices);
]];

local pcl_IntegralImageNormalEstimation_declaration = [[
typedef struct {} IntegralImageNormalEstimation_TYPE_KEY;
IntegralImageNormalEstimation_TYPE_KEY* pcl_IntegralImageNormalEstimation_TYPE_KEY_new();
void pcl_IntegralImageNormalEstimation_TYPE_KEY_delete(IntegralImageNormalEstimation_TYPE_KEY *self);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setInputCloud(IntegralImageNormalEstimation_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setRectSize(IntegralImageNormalEstimation_TYPE_KEY *self, int width, int height);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setMaxDepthChangeFactor(IntegralImageNormalEstimation_TYPE_KEY *self, float max_depth_change_factor);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setBorderPolicy(IntegralImageNormalEstimation_TYPE_KEY *self, int border_policy);
Normal pcl_IntegralImageNormalEstimation_TYPE_KEY_computePointNormal(IntegralImageNormalEstimation_TYPE_KEY *self, int pos_x, int pos_y, unsigned int point_index);
Normal pcl_IntegralImageNormalEstimation_TYPE_KEY_computePointNormalMirror(IntegralImageNormalEstimation_TYPE_KEY *self, int pos_x, int pos_y, unsigned int point_index);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setNormalSmoothingSize(IntegralImageNormalEstimation_TYPE_KEY *self, float normal_smoothing_size);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setNormalEstimationMethod(IntegralImageNormalEstimation_TYPE_KEY *self, int normal_estimation_method);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setDepthDependentSmoothing(IntegralImageNormalEstimation_TYPE_KEY *self, bool use_depth_dependent_smoothing);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_getViewPoint(IntegralImageNormalEstimation_TYPE_KEY *self, THFloatTensor *out_pt);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_setViewPoint(IntegralImageNormalEstimation_TYPE_KEY *self, THFloatTensor *pt);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_useSensorOriginAsViewPoint(IntegralImageNormalEstimation_TYPE_KEY *self);
void pcl_IntegralImageNormalEstimation_TYPE_KEY_compute(IntegralImageNormalEstimation_TYPE_KEY *self, PointCloud_Normal *output);
]];

local pcl_PCLVisualizer_declaration = [[
typedef struct {} PCLVisualizer;
PCLVisualizer *pcl_PCLVisualizer_new(const char *name, bool create_interactor);
void pcl_PCLVisualizer_delete(PCLVisualizer *self);
void pcl_PCLVisualizer_setFullScreen(PCLVisualizer *self, bool mode);
void pcl_PCLVisualizer_setWindowName(PCLVisualizer *self, const char *name);
void pcl_PCLVisualizer_setWindowBorders(PCLVisualizer *self, bool mode);
void pcl_PCLVisualizer_setPosition(PCLVisualizer *self, int x, int y);
void pcl_PCLVisualizer_spin(PCLVisualizer *self);
void pcl_PCLVisualizer_spinOnce(PCLVisualizer *self, int time, bool force_redraw);
void pcl_PCLVisualizer_addCoordinateSystem(PCLVisualizer *self, double scale, const char *id, int viewport);
void pcl_PCLVisualizer_addCoordinateSystemPose(PCLVisualizer *self, double scale, THFloatTensor *transform, const char *id, int viewport);
bool pcl_PCLVisualizer_updateCoordinateSystemPose(PCLVisualizer *self, const char *id, THFloatTensor *pose);
int  pcl_PCLVisualizer_createViewPort(PCLVisualizer *self, double xmin, double ymin, double xmax, double ymax);
void pcl_PCLVisualizer_createViewPortCamera(PCLVisualizer *self, int viewport);
void pcl_PCLVisualizer_setBackgroundColor(PCLVisualizer *self, double r, double g, double b, int viewport);
bool pcl_PCLVisualizer_updatePointCloudPose(PCLVisualizer *self, const char *id, THFloatTensor *transform);
bool pcl_PCLVisualizer_updateShapePose(PCLVisualizer *self, const char *id, THFloatTensor *transform);
bool pcl_PCLVisualizer_removeAllPointClouds(PCLVisualizer *self, int viewport);
bool pcl_PCLVisualizer_removePointCloud(PCLVisualizer *self, const char *id, int viewport);
bool pcl_PCLVisualizer_removeAllShapes(PCLVisualizer *self, int viewport);
bool pcl_PCLVisualizer_removeShape(PCLVisualizer *self, const char *id, int viewport);
bool pcl_PCLVisualizer_removeAllCoordinateSystems(PCLVisualizer *self, int viewport);
bool pcl_PCLVisualizer_removeCoordinateSystem(PCLVisualizer *self, const char *id, int viewport);
bool pcl_PCLVisualizer_addText1(PCLVisualizer *self, const char *text, int xpos, int ypos, const char *id, int viewport);
bool pcl_PCLVisualizer_addText2(PCLVisualizer *self, const char *text, int xpos, int ypos, double r, double g, double b, const char *id, int viewport);
bool pcl_PCLVisualizer_addText3(PCLVisualizer *self, const char *text, int xpos, int ypos, int fontsize, double r, double g, double b, const char *id, int viewport);
bool pcl_PCLVisualizer_removeText3D(PCLVisualizer *self, const char *id, int viewport);
bool pcl_PCLVisualizer_contains(PCLVisualizer *self, const char *id);
void pcl_PCLVisualizer_initCameraParameters(PCLVisualizer *self);
bool pcl_PCLVisualizer_setPointCloudRenderingProperties1(PCLVisualizer *self, int property, double value, const char *id, int viewport);
bool pcl_PCLVisualizer_setPointCloudRenderingProperties3(PCLVisualizer *self, int property, double val1, double val2, double val3, const char *id, int viewport);
bool pcl_PCLVisualizer_wasStopped(PCLVisualizer *self);
void pcl_PCLVisualizer_resetStoppedFlag(PCLVisualizer *self);
void pcl_PCLVisualizer_close(PCLVisualizer *self);
void pcl_PCLVisualizer_setShowFPS(PCLVisualizer *self, bool show_fps);
void pcl_PCLVisualizer_updateCamera(PCLVisualizer *self);
void pcl_PCLVisualizer_resetCamera(PCLVisualizer *self);
void pcl_PCLVisualizer_resetCameraViewpoint(PCLVisualizer *self, const char *id);
void pcl_PCLVisualizer_setCameraPosition(PCLVisualizer *self, double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z, double up_x, double up_y, double up_z, int viewport);
void pcl_PCLVisualizer_setCameraClipDistances(PCLVisualizer *self, double near, double far, int viewport);
void pcl_PCLVisualizer_setCameraFieldOfView(PCLVisualizer *self, double fovy, int viewport);
void pcl_PCLVisualizer_setCameraParameters_Tensor(PCLVisualizer *self, THFloatTensor *intrinsics, THFloatTensor *extrinsics, int viewport);
void pcl_PCLVisualizer_setSize(PCLVisualizer *self, int xw, int yw);

void pcl_PCLVisualizer_saveScreenshot(PCLVisualizer *self, const char *fn);
void pcl_PCLVisualizer_saveCameraParameters(PCLVisualizer *self, const char *fn);
bool pcl_PCLVisualizer_loadCameraParameters(PCLVisualizer *self, const char *fn);
void pcl_PCLVisualizer_getViewerPose(PCLVisualizer *self, int viewport, THFloatTensor *result);
bool pcl_PCLVisualizer_addPlane_Coefficients(PCLVisualizer *self, THFloatTensor *coefficients, double x, double y, double z, const char *id, int viewport);
bool pcl_PCLVisualizer_addLine_Coefficients(PCLVisualizer *self, THFloatTensor *coefficients, const char *id, int viewport);
bool pcl_PCLVisualizer_addSphere_Coefficients(PCLVisualizer *self, THFloatTensor *coefficients, const char *id, int viewport);
bool pcl_PCLVisualizer_addCube_Coefficients(PCLVisualizer *self, THFloatTensor *coefficients, const char *id, int viewport);
bool pcl_PCLVisualizer_addCylinder_Coefficients(PCLVisualizer *self, THFloatTensor *coefficients, const char *id, int viewport);
bool pcl_PCLVisualizer_addCone_Coefficients(PCLVisualizer *self, THFloatTensor *coefficients, const char *id, int viewport);
bool pcl_PCLVisualizer_addCube(PCLVisualizer *self, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, double r, double g, double b, const char *id, int viewport);
void pcl_PCLVisualizer_setRepresentationToSurfaceForAllActors(PCLVisualizer *self);
void pcl_PCLVisualizer_setRepresentationToPointsForAllActors(PCLVisualizer *self);
void pcl_PCLVisualizer_setRepresentationToWireframeForAllActors(PCLVisualizer *self);

typedef struct {} event_connection;
typedef void (*KeyboardEventCallback)(bool keydown, const char *key_sym, int key_code, bool alt, bool ctrl, bool shift);
typedef void (*MouseEventCallback)(int type, int button, int x, int y, bool alt, bool ctrl, bool shift, bool selection_mode);
typedef void (*PointPickingCallback)(int idx1, int idx2, float x1, float y1, float z1, float x2, float y2, float z2);
typedef void (*AreaPickingCallback)(Indices *indices);
event_connection *pcl_PCLVisualizer_registerKeyboardCallback(PCLVisualizer *self, KeyboardEventCallback callback);
event_connection *pcl_PCLVisualizer_registerMouseCallback(PCLVisualizer *self, MouseEventCallback callback);
event_connection *pcl_PCLVisualizer_registerPointPickingCallback(PCLVisualizer *self, PointPickingCallback callback);
event_connection *pcl_PCLVisualizer_registerAreaPickingCallback(PCLVisualizer *self, AreaPickingCallback callback);
void pcl_PCLVisualizer_unregisterCallback(event_connection *connection);
void pcl_PCLVisualizer_deleteCallback(event_connection *connection);
]]

local pcl_PCLVisualizer_template_declaration = [[
typedef struct {} PointCloudColorHandler;

bool pcl_PCLVisualizer_TYPE_KEY_addPointCloud(PCLVisualizer *self, PointCloud_TYPE_KEY *cloud, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_addPointCloudWithColorHandler(PCLVisualizer *self, PointCloud_TYPE_KEY *cloud, PointCloudColorHandler *color_handler, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_addPointCloudNormals(PCLVisualizer *self, PointCloud_TYPE_KEY *cloud, int level, float scale, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_addPointCloudNormals2(PCLVisualizer *self, PointCloud_TYPE_KEY *cloud, PointCloud_Normal *normals, int level, float scale, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_updatePointCloud(PCLVisualizer *self, PointCloud_TYPE_KEY *cloud, const char *id);
bool pcl_PCLVisualizer_TYPE_KEY_addCorrespondences(PCLVisualizer *self, PointCloud_TYPE_KEY *source_points, PointCloud_TYPE_KEY *target_points, Correspondences *correspondences, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_updateCorrespondences(PCLVisualizer *self, PointCloud_TYPE_KEY *source_points, PointCloud_TYPE_KEY *target_points, Correspondences *correspondences, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_addLine(PCLVisualizer *self, const PointTYPE_KEY &pt1, const PointTYPE_KEY &pt2, double r, double g, double b, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_addSphere(PCLVisualizer *self, const PointTYPE_KEY &center, double radius, double r, double g, double b, const char *id, int viewport);
bool pcl_PCLVisualizer_TYPE_KEY_updateSphere(PCLVisualizer *self, const PointTYPE_KEY &center, double radius, double r, double g, double b, const char *id);
bool pcl_PCLVisualizer_TYPE_KEY_addText3D(PCLVisualizer *self, const char *text, const PointTYPE_KEY &position, double textScale, double r, double g, double b, const char *id, int viewport);

PointCloudColorHandler *pcl_PCLVisualizer_TYPE_KEY_createColorHandlerRandom(PointCloud_TYPE_KEY *cloud);
PointCloudColorHandler *pcl_PCLVisualizer_TYPE_KEY_createColorHandlerCustom(PointCloud_TYPE_KEY *cloud, double r, double g, double b);
PointCloudColorHandler *pcl_PCLVisualizer_TYPE_KEY_createColorHandlerGenericField(PointCloud_TYPE_KEY *cloud, const char *field_name);
void pcl_PCLVisualizer_TYPE_KEY_deleteColorHandler(PointCloudColorHandler *handler);
]]

ffi.cdef(pcl_PCLVisualizer_declaration)

local pcl_PCLPointCloud2_declarations = [[
pcl_PCLPointCloud2* pcl_PCLPointCloud2_new();
void pcl_PCLPointCloud2_delete(pcl_PCLPointCloud2 *self);
void pcl_PCLPointCloud2_getFieldNames(pcl_PCLPointCloud2 *self, THByteStorage *string_buffer);
void pcl_PCLPointCloud2_tostring(pcl_PCLPointCloud2 *self, THByteStorage *string_buffer);
]]

ffi.cdef(pcl_PCLPointCloud2_declarations)

local supported_keys = { 'XYZ', 'XYZI', 'XYZRGBA', 'XYZNormal', 'XYZINormal', 'XYZRGBNormal' }
local declarations = {
    pcl_PointCloud_declaration,
    KdTreeFLANN_declarations,
    generic_declarations,
    pcl_CorrespondenceEstimation_declaration,
    pcl_SampleConsensusPrerejective_declaration,
    pcl_BoundaryEstimation_declaration,
    pcl_SACSegmentation_declaration,
    pcl_EuclideanClusterExtraction_declaration,
    pcl_PCLVisualizer_template_declaration,
    pcl_OrganizedEdge_declaration,
    pcl_IntegralImageNormalEstimation_declaration
  }
for i,v in ipairs(supported_keys) do
  for j,declaration in ipairs(declarations) do
    local specialized = string.gsub(declaration, 'TYPE_KEY', v)
    ffi.cdef(specialized)
  end
end

local function register_special_types(names, declarations)
  for i,n in ipairs(names) do
    for j,declaration in ipairs(declarations) do
      local specialized = string.gsub(declaration, 'PointTYPE_KEY', n)
      specialized = string.gsub(specialized, 'TYPE_KEY', n)
      ffi.cdef(specialized)
    end
  end
end

local function add_additional_point_types()
  local names = { 'FPFHSignature33', 'VFHSignature308' }
  local declarations = {
    pcl_PointCloud_declaration,
    KdTreeFLANN_declarations,
    pcl_CorrespondenceEstimation_declaration,
    pcl_SampleConsensusPrerejective_declaration
  }
  register_special_types(names, declarations)
end
add_additional_point_types()

local function add_additional_point_cloud_types()
  local names = { 'Normal', 'Boundary', 'Label' }
  local declarations = {
    pcl_PointCloud_declaration
  }
  register_special_types(names, declarations)
end
add_additional_point_cloud_types()

local specialized_declarations =
[[
void pcl_PointCloud_XYZRGBA_readRGBAfloat(void *cloud, struct THFloatTensor *output);
void pcl_PointCloud_XYZRGBA_readRGBAbyte(void *cloud, struct THByteTensor *output);
void pcl_PointCloud_XYZRGBA_writeRGBAfloat(void *cloud, struct THFloatTensor *input);
void pcl_PointCloud_XYZRGBA_writeRGBAbyte(void *cloud, struct THByteTensor *input);
void pcl_PointCloud_XYZRGBA_writeRGBfloat(void *cloud, struct THFloatTensor *input, bool setAlpha, float alpha);
void pcl_PointCloud_XYZRGBA_writeRGBbyte(void *cloud, struct THByteTensor *input, bool setAlpha, uint8_t alpha);

void pcl_PointCloud_XYZ_addNormals(PointCloud_XYZ *self, PointCloud_Normal *normals, PointCloud_XYZNormal *output);
void pcl_PointCloud_XYZI_addNormals(PointCloud_XYZI *self, PointCloud_Normal *normals, PointCloud_XYZINormal *output);
void pcl_PointCloud_XYZRGBA_addNormals(PointCloud_XYZRGBA*self, PointCloud_Normal *normals, PointCloud_XYZRGBNormal *output);

void pcl_PointCloud_Normal_copyNormal(PointCloud_Normal *cloud_in, Indices *indices, PointCloud_Normal *cloud_out);
void pcl_PointCloud_Normal_copyXYZNormal(PointCloud_Normal *cloud_in, Indices *indices, PointCloud_XYZNormal *cloud_out);
void pcl_PointCloud_Normal_copyXYZINormal(PointCloud_Normal *cloud_in, Indices *indices, PointCloud_XYZINormal *cloud_out);
void pcl_PointCloud_Normal_copyXYZRGBNormal(PointCloud_Normal *cloud_in, Indices *indices, PointCloud_XYZRGBNormal *cloud_out);
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
  'PointXYZRGBNormal',    -- float x, y, z, rgb, normal[3], curvature;
  'PointXYZINormal',      -- float x, y, z, intensity, normal[3], curvature;
  'FPFHSignature33',
  'VFHSignature308',
  'Boundary',
  'Label'
}

local nameByType = {}
pcl.pointTypeByName = {}
pcl.nameOfPointType = {}

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
  pcl.nameOfPointType[t] = n
  n = string.gsub(n, 'Point', '')
  pcl.pointTypeByName[string.lower(n)] = t
end

-- alias for PointNormal
pcl.XYZNormal = pcl.PointNormal
pcl.pointTypeByName['pointxyznormal'] = pcl.PointNormal
pcl.pointTypeByName['xyznormal'] = pcl.PointNormal

pcl.Correspondence = ffi.typeof('Correspondence')

function pcl.getPointTypeName(pointType)
  return pcl.nameOfPointType[pcl.pointType(pointType)]
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

local function createpairs_n(fields)
  return function(self)
    local k = nil
    return function()
      local _
      k, n = next(fields, k)
      local v = self[n]
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

-- Correspondence metatype
local Correspondence = {
  prototype = {
    set = function(self, v) ffi.copy(self, v, ffi.sizeof(pcl.Correspondence)) end
  },
  fields = { 'index_query', 'index_match', 'distance' }
}
Correspondence.__pairs = createpairs_n(Correspondence.fields)
function Correspondence:__index(i) return Correspondence.prototype[i] end
function Correspondence:__tostring() return string.format('{ index_query: %d, index_match: %d, distance: %f }', self.index_query, self.index_match, self.distance) end
ffi.metatype(pcl.Correspondence, Correspondence)
pcl.metatype[pcl.Correspondence] = Correspondence

-- PointXYZ metatype
local PointXYZ = {
  prototype = {
    totensor = totensor,
    set = set,
    hasNormal = false,
    type = pcl.PointXYZ
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
    hasNormal = false,
    type = pcl.PointXYZI
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
    hasNormal = false,
    type = pcl.PointXYZRGBA
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
    hasNormal = true,
    type = pcl.Normal
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
    hasNormal = true,
    type = pcl.PointNormal
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
    hasNormal = true,
    type = pcl.PointXYZINormal
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

-- PointXYZRGBNormal metatype
local PointXYZRGBNormal = {
  prototype = {
    tensor = totensor,
    set = set,
    hasNormal = true,
    type = pcl.PointXYZRGBNormal
  },
  __eq = eq,
  __len = len,
  fields = { 'x', 'y', 'z', 'w', 'normal_x', 'normal_y', 'normal_z', '_1', 'rgba', 'curvature', '_2', '_3' }
}
PointXYZRGBNormal.__pairs = createpairs(PointXYZRGBNormal.fields)
function PointXYZRGBNormal:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZRGBNormal.prototype[i] end end
function PointXYZRGBNormal:__newindex(i, v) if i > 0 and i <= #self then self.data[i-1] = v else error('index out of range') end end
function PointXYZRGBNormal:__tostring()
  return string.format('{ x:%f, y:%f, z:%f, normal_x:%f, normal_y:%f, normal_z:%f, rgba:%08X, curvature:%f }',
    self.x, self.y, self.z, self.normal_x, self.normal_y, self.normal_z, self.rgba, self.curvature)
end
ffi.metatype(pcl.PointXYZRGBNormal, PointXYZRGBNormal)
pcl.metatype[pcl.PointXYZRGBNormal] = PointXYZRGBNormal

local FPFHSignature33 = {
  __len = len
}
function FPFHSignature33:__index(i) if type(i) == "number" then return self.histogram[i-1] end end
function FPFHSignature33:__newindex(i, v) if i > 0 and i <= #self then self.histogram[i-1] = v else error('index out of range') end end
function FPFHSignature33:set(v) ffi.copy(self, v, ffi.sizeof(pcl.FPFHSignature33)) end
ffi.metatype(pcl.FPFHSignature33, FPFHSignature33)
pcl.metatype[pcl.FPFHSignature33] = FPFHSignature33

local Boundary = {
  fields = { 'boundary_point' }
}
function Boundary:set(v) ffi.copy(self, v, ffi.sizeof(pcl.Boundary)) end
function Boundary:__tostring() return string.format('{ boundary_point: %d }', self.boundary_point) end
ffi.metatype(pcl.Boundary, Boundary)
pcl.metatype[pcl.Boundary] = Boundary

local Label = {
  fields = { 'label' }
}
function Label:set(v) ffi.copy(self, v, ffi.sizeof(pcl.Label)) end
function Label:__tostring() return string.format('{ label: %d }', self.label) end
ffi.metatype(pcl.Label, Label)
pcl.metatype[pcl.Label] = Label

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
    min = -2147483648,
    max =  2147483647
  },
  uint32 = {
    min =  0,
    max =  0xffffffff
  },
  int64 = {
    min = -9223372036854775808,
    max =  9223372036854775807
  },
  uint64 = {
    min =  0,
    max =  0xffffffffffffffff
  }
}

pcl.COLORS = {
  WHITE   = { 1, 1, 1, 1 },
  SILVER  = { 0.75, 0.75, 0.75, 1 },
  GRAY    = { 0.5, 0.5, 0.5, 1 },
  BLACK   = { 0, 0, 0, 1 },
  RED     = { 1, 0, 0, 1 },
  MAROON  = { 0.5, 0, 0, 1 },
  YELLOW  = { 1, 1, 0, 1 },
  OLIVE   = { 0.5, 0.5, 0, 1 },
  LIME    = { 0, 1, 0, 1 },
  GREEN   = { 0, 0.5, 0, 1 },
  AQUA    =	{ 0, 1, 1, 1 },
  TEAL    = { 0, 0.5, 0.5, 1 },
  BLUE    = { 0, 0, 1, 1 },
  NAVY    = { 0, 0, 0.5, 1 },
  FUCHSIA = { 1, 0, 1, 1 },
  PURPLE  = { 0.5, 0, 0.5, 1 }
}

return pcl
