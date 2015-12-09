local ffi = require 'ffi'

local pcl = {}

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
typedef struct Normal { "..PCL_POINT4D.." union { struct { float curvature; }; float data_c[4]; }; } Normal; \z
typedef struct Axis { "..PCL_NORMAL4D.." } Axis; \z
typedef struct PointNormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float curvature; }; float data_c[4]; }; } PointNormal; \z
typedef struct PointXYZRGBNormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { "..PCL_RGB.." float curvature; }; float data_c[4]; }; } PointXYZRGBNormal; \z
typedef struct PointXYZINormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float intensity; float curvature; }; float data_c[4]; }; } PointXYZINormal; \z
typedef struct _PointsBuffer { THFloatStorage* storage; uint32_t width, height, dim; } _PointsBuffer;" ..
[[
typedef struct PointCloud_XYZ {} PointCloud_XYZ;
typedef struct PointCloud_XYZI {} PointCloud_XYZI;
typedef struct PointCloud_XYZRGBA {} PointCloud_XYZRGBA;

void* pcl_CloudViewer_new(const char *window_name);
void pcl_CloudViewer_delete(void *self);
bool pcl_CloudViewer_wasStopped(void *self, int millis_to_wait);

void* pcl_OpenNI2Stream_XYZRGBA_new(const char* device_id, int max_backlog);
void pcl_OpenNI2Stream_XYZRGBA_delete(void* self);
void pcl_OpenNI2Stream_XYZRGBA_start(void* self);
void pcl_OpenNI2Stream_XYZRGBA_stop(void* self);
void* pcl_OpenNI2Stream_XYZRGBA_read(void* self, int timeout_milliseconds);

void pcl_Primitive_XYZ_createSphere(PointCloud_XYZ *output, double radius, double thetaRes, double phiRes, int samples, float resolution);
void pcl_Primitive_XYZ_createCube(PointCloud_XYZ *output, double x, double y, double z, int samples, float resolution);
void pcl_Primitive_XYZ_createCylinder(PointCloud_XYZ *output, double height, double radius, int facets, int samples, float resolution);
void pcl_Primitive_XYZ_createCone(PointCloud_XYZ *output, double height, double radius, int facets, int samples, float resolution);
void pcl_Primitive_XYZ_createPlatonicSolid(PointCloud_XYZ *output, int solidType, int samples, float resolution);
void pcl_Primitive_XYZ_createPlane(PointCloud_XYZ *output, double x1, double y1, double z1, double x2, double y2, double z2, int samples, float resolution);
void pcl_Primitive_XYZ_createDisk(PointCloud_XYZ *output, double innerRadius, double outerRadius, int radialResolution,int samples, float resolution);
]]
ffi.cdef(cdef)

local generic_declarations = 
[[
PointCloud_TYPE_KEY* pcl_PointCloud_TYPE_KEY_new(uint32_t width, uint32_t height);
PointCloud_TYPE_KEY* pcl_PointCloud_TYPE_KEY_clone(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_delete(PointCloud_TYPE_KEY *self);
uint32_t pcl_PointCloud_TYPE_KEY_width(PointCloud_TYPE_KEY *self);
uint32_t pcl_PointCloud_TYPE_KEY_height(PointCloud_TYPE_KEY *self);
bool pcl_PointCloud_TYPE_KEY_isDense(PointCloud_TYPE_KEY *self);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at1D(PointCloud_TYPE_KEY *self, int n);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at2D(PointCloud_TYPE_KEY *self, int column, int row);
void pcl_PointCloud_TYPE_KEY_clear(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_reserve(PointCloud_TYPE_KEY *self, size_t n);
uint32_t pcl_PointCloud_TYPE_KEY_size(PointCloud_TYPE_KEY *self);
bool pcl_PointCloud_TYPE_KEY_empty(PointCloud_TYPE_KEY *self);
bool pcl_PointCloud_TYPE_KEY_isOrganized(PointCloud_TYPE_KEY* self);
void pcl_PointCloud_TYPE_KEY_push_back(PointCloud_TYPE_KEY* self, const PointTYPE_KEY& pt);
void pcl_PointCloud_TYPE_KEY_insert(PointCloud_TYPE_KEY* self, size_t position, const PointTYPE_KEY& pt, size_t n);
void pcl_PointCloud_TYPE_KEY_erase(PointCloud_TYPE_KEY* self, size_t begin, size_t end);
_PointsBuffer pcl_PointCloud_TYPE_KEY_points(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_add(PointCloud_TYPE_KEY *self, PointCloud_TYPE_KEY *other);
THFloatStorage *pcl_PointCloud_TYPE_KEY_sensorOrigin(PointCloud_TYPE_KEY *self);
THFloatStorage *pcl_PointCloud_TYPE_KEY_sensorOrientation(PointCloud_TYPE_KEY *self);
void pcl_PointCloud_TYPE_KEY_transform(PointCloud_TYPE_KEY *self, THFloatTensor *mat, PointCloud_TYPE_KEY *output);
void pcl_PointCloud_TYPE_KEY_getMinMax3D(PointCloud_TYPE_KEY *self, PointTYPE_KEY& min, PointTYPE_KEY& max);
void pcl_PointCloud_TYPE_KEY_fromPCLPointCloud2(void *cloud, void *msg);
void pcl_PointCloud_TYPE_KEY_toPCLPointCloud2(void *cloud, void *msg);
int pcl_PointCloud_TYPE_KEY_loadPCDFile(PointCloud_TYPE_KEY *cloud, const char *fn);
int pcl_PointCloud_TYPE_KEY_savePCDFile(PointCloud_TYPE_KEY *cloud, const char *fn, bool binary);
int pcl_PointCloud_TYPE_KEY_loadPLYFile(PointCloud_TYPE_KEY *cloud, const char *fn);
int pcl_PointCloud_TYPE_KEY_savePLYFile(PointCloud_TYPE_KEY *cloud, const char *fn, bool binary);
int pcl_PointCloud_TYPE_KEY_loadOBJFile(PointCloud_TYPE_KEY *cloud, const char *fn);
void pcl_PointCloud_TYPE_KEY_savePNGFile(PointCloud_TYPE_KEY *cloud, const char *fn, const char* field_name);
int pcl_PointCloud_TYPE_KEY_readXYZfloat(PointCloud_TYPE_KEY *cloud, struct THFloatTensor *output);

void pcl_CloudViewer_TYPE_KEY_showCloud(void *self, PointCloud_TYPE_KEY *cloud, const char *cloudname);

typedef struct PCA_TYPE_KEY {} PCA_TYPE_KEY;
PCA_TYPE_KEY* pcl_PCA_TYPE_KEY_new(bool basis_only);
PCA_TYPE_KEY* pcl_PCA_TYPE_KEY_clone(PCA_TYPE_KEY *self);
void pcl_PCA_TYPE_KEY_delete(PCA_TYPE_KEY *self);
void pcl_PCA_TYPE_KEY_set_inputCloud(PCA_TYPE_KEY *self, PointCloud_TYPE_KEY *cloud);
void pcl_PCA_TYPE_KEY_get_mean(PCA_TYPE_KEY *self, struct THFloatTensor* output);
void pcl_PCA_TYPE_KEY_get_eigenVectors(PCA_TYPE_KEY *self, struct THFloatTensor *output);
void pcl_PCA_TYPE_KEY_get_eigenValues(PCA_TYPE_KEY *self, struct THFloatTensor *output);
void pcl_PCA_TYPE_KEY_get_coefficients(PCA_TYPE_KEY *self, struct THFloatTensor* output);
void pcl_PCA_TYPE_KEY_project_cloud(PCA_TYPE_KEY *self, PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output);
void pcl_PCA_TYPE_KEY_reconstruct_cloud(PCA_TYPE_KEY *self, PointCloud_TYPE_KEY *input, PointCloud_TYPE_KEY *output);

typedef struct ICP_TYPE_KEY {} ICP_TYPE_KEY;
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
]]

local supported_keys = { 'XYZ', 'XYZI', 'XYZRGBA' }
for i,v in ipairs(supported_keys) do
  local specialized = string.gsub(generic_declarations, 'TYPE_KEY', v)
  ffi.cdef(specialized)
end

local specialized_declarations = 
[[
int pcl_PointCloud_XYZRGBA_readRGBAfloat(void *cloud, struct THFloatTensor *output);
int pcl_PointCloud_XYZRGBA_readRGBAbyte(void *cloud, struct THByteTensor *output);
]]
ffi.cdef(specialized_declarations)

pcl.lib = ffi.load(package.searchpath('libpcl', package.cpath))

local pointTypeNames = { 
  'PointXYZ',             -- float x, y, z;
  'PointXYZI',            -- float x, y, z, intensity;
  'PointXYZRGBA',         -- float x, y, z; uint32_t rgba;
  'PointXY',              -- float x, y;
  'Normal',               -- float normal[3], curvature;
  'PointNormal',          -- float x, y, z; float normal[3], curvature;
  'PointXYZRGBNormal',    -- float x, y, z, rgb, normal[3], curvature;
  'PointXYZINormal'       -- float x, y, z, intensity, normal[3], curvature;
}

local nameByType = {}
pcl.pointTypeByName = {}

function pcl.pointType(pointType)
  if type(pointType) == 'string' then
    pointType = pcl.pointTypeByName[string.lower(pointType)]
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

-- PointXYZRGBA metatype
local PointXYZRGBA = {
  prototype = {
    tensor = totensor,
    set = set,
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
  int32 = {
    min = −2147483648,
    max =  2147483647
  },
  int64 = {
    min = −9223372036854775808,
    max =  9223372036854775807
  }    
}

return pcl
