local ffi = require 'ffi'

local pcl = {}

local PCL_POINT4D = "union __attribute__((aligned(16))) { struct { float x; float y; float z; }; float data[4]; };"
local PCL_NORMAL4D = "union __attribute__((aligned(16))) { struct { float normal_x; float normal_y; float normal_z; }; float normal[3]; float data_n[4]; };"
local PCL_RGB = "union { union { struct { uint8_t b; uint8_t g; uint8_t r; uint8_t a; }; float rgb; }; uint32_t rgba; };"

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
void* pcl_CloudViewer_new(const char *window_name);
void pcl_CloudViewer_delete(void *self);
bool pcl_CloudViewer_wasStopped(void *self, int millis_to_wait);

void* pcl_OpenNI2Stream_XYZRGBA_new(const char* device_id, int max_backlog);
void pcl_OpenNI2Stream_XYZRGBA_delete(void* self);
void pcl_OpenNI2Stream_XYZRGBA_start(void* self);
void pcl_OpenNI2Stream_XYZRGBA_stop(void* self);
void* pcl_OpenNI2Stream_XYZRGBA_read(void* self, int timeout_milliseconds);
]]
ffi.cdef(cdef)

local generic_declarations = 
[[
void* pcl_PointCloud_TYPE_KEY_new(uint32_t width, uint32_t height);
void* pcl_PointCloud_TYPE_KEY_clone(void *self);
void pcl_PointCloud_TYPE_KEY_delete(void *self);
uint32_t pcl_PointCloud_TYPE_KEY_width(void *self);
uint32_t pcl_PointCloud_TYPE_KEY_height(void *self);
bool pcl_PointCloud_TYPE_KEY_isDense(void *self);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at1D(void *self, int n);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at2D(void *self, int column, int row);
void pcl_PointCloud_TYPE_KEY_clear(void *self);
bool pcl_PointCloud_TYPE_KEY_empty(void *self);
bool pcl_PointCloud_TYPE_KEY_isOrganized(void* self);
_PointsBuffer pcl_PointCloud_TYPE_KEY_points(void *self);
void pcl_PointCloud_TYPE_KEY_add(void *self, void *other);
THFloatStorage* pcl_PointCloud_TYPE_KEY_sensorOrigin(void *self);
THFloatStorage* pcl_PointCloud_TYPE_KEY_sensorOrientation(void *self);
void pcl_PointCloud_TYPE_KEY_fromPCLPointCloud2(void *cloud, void *msg);
void pcl_PointCloud_TYPE_KEY_toPCLPointCloud2(void *cloud, void *msg);
int pcl_PointCloud_TYPE_KEY_loadPCDFile(void *cloud, const char *fn);
int pcl_PointCloud_TYPE_KEY_savePCDFile(void *cloud, const char *fn, bool binary);
int pcl_PointCloud_TYPE_KEY_loadPLYFile(void *cloud, const char *fn);
int pcl_PointCloud_TYPE_KEY_savePLYFile(void *cloud, const char *fn, bool binary);
int pcl_PointCloud_TYPE_KEY_loadOBJFile(void *cloud, const char *fn);
void pcl_PointCloud_TYPE_KEY_savePNGFile(void *cloud, const char *fn, const char* field_name);
int pcl_PointCloud_TYPE_KEY_readXYZfloat(void *cloud, struct THFloatTensor *output);

void pcl_CloudViewer_TYPE_KEY_showCloud(void *self, void *cloud, const char *cloudname);

typedef struct PCA_TYPE_KEY {} PCA_TYPE_KEY;
PCA_TYPE_KEY* pcl_PCA_TYPE_KEY_new(bool basis_only);
PCA_TYPE_KEY* pcl_PCA_TYPE_KEY_clone(PCA_TYPE_KEY *self);
void pcl_PCA_TYPE_KEY_delete(PCA_TYPE_KEY *self);
void pcl_PCA_TYPE_KEY_set_inputCloud(PCA_TYPE_KEY *self, void *cloud);
void pcl_PCA_TYPE_KEY_get_mean(PCA_TYPE_KEY *self, struct THFloatTensor* output);
void pcl_PCA_TYPE_KEY_get_eigenVectors(PCA_TYPE_KEY *self, struct THFloatTensor *output);
void pcl_PCA_TYPE_KEY_get_eigenValues(PCA_TYPE_KEY *self, struct THFloatTensor *output);
void pcl_PCA_TYPE_KEY_get_coefficients(PCA_TYPE_KEY *self, struct THFloatTensor* output);
void pcl_PCA_TYPE_KEY_project_cloud(PCA_TYPE_KEY *self, void *input, void *output);
void pcl_PCA_TYPE_KEY_reconstruct_cloud(PCA_TYPE_KEY *self, void *input, void *output);
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

pcl.PointXYZ            = ffi.typeof('PointXYZ')            -- float x, y, z;
pcl.PointXYZI           = ffi.typeof('PointXYZI')           -- float x, y, z, intensity;
pcl.PointXYZRGBA        = ffi.typeof('PointXYZRGBA')        -- float x, y, z; uint32_t rgba;
pcl.PointXY             = ffi.typeof('PointXY')             -- float x, y;
pcl.Normal              = ffi.typeof('Normal')              -- float normal[3], curvature;
pcl.PointNormal         = ffi.typeof('PointNormal')         -- float x, y, z; float normal[3], curvature;
pcl.PointXYZRGBNormal   = ffi.typeof('PointXYZRGBNormal')   -- float x, y, z, rgb, normal[3], curvature;
pcl.PointXYZINormal     = ffi.typeof('PointXYZINormal')     -- float x, y, z, intensity, normal[3], curvature;

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

-- PointXYZ metatype
local PointXYZ = {
  prototype = {
    fromtensor = function(t) 
      local p = pcl.PointXYZ() 
      for i=1,3 do p[i] = t[i] end
      return p
    end,
    totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z }) end,
    set = function(self, v) ffi.copy(self, v, ffi.sizeof(pcl.PointXYZ)) end
  },
  __len = function(self) return 3 end,
  fields = { 'x', 'y', 'z' }
}

PointXYZ.__pairs = createpairs(PointXYZ.fields)
function PointXYZ:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZ.prototype[i] end end
function PointXYZ:__newindex(i, v) self.data[i-1] = v end
function PointXYZ:__tostring() return string.format('{ x:%f, y:%f, z:%f }', self.x, self.y, self.z) end 
ffi.metatype("PointXYZ", PointXYZ)

-- PointXYZI metatype
local PointXYZI = {
  prototype = {
    fromtensor = function(t) 
      local p = pcl.PointXYZI() 
      for i=1,4 do p[i] = t[i] end
      return p
    end,
    totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z, self.intensity }) end,
    set = function(self, v) ffi.copy(self, v, ffi.sizeof(pcl.PointXYZI)) end
  },
  __len = function(self) return 4 end,
  fields = { 'x', 'y', 'z', 'i' }
}

PointXYZI.__pairs = createpairs(PointXYZI.fields)
function PointXYZI:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZI.prototype[i] end end
function PointXYZI:__newindex(i, v) self.data[i-1] = v end
function PointXYZI:__tostring() return string.format('{ x:%f, y:%f, z:%f, intensity:%f }', self.x, self.y, self.z, self.intensity) end 
ffi.metatype("PointXYZI", PointXYZI)

-- PointXYZRGBA metatype
local PointXYZRGBA = {
  prototype = {
    fromtensor = function(t) 
      local p = pcl.PointXYZI() 
      for i=1,4 do p[i] = t[i] end
      return p
    end,
    totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z, self.rgb }) end,
    set = function(self, v) ffi.copy(self, v, ffi.sizeof(pcl.PointXYZRGBA)) end
  },
  __len = function(self) return 4 end,
  fields = { 'x', 'y', 'z', 'rgba' }
}

PointXYZRGBA.__pairs = createpairs(PointXYZRGBA.fields)
function PointXYZRGBA:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZRGBA.prototype[i] end end
function PointXYZRGBA:__newindex(i, v) self.data[i-1] = v end
function PointXYZRGBA:__tostring() return string.format('{ x:%f, y:%f, z:%f, rgba:%08X }', self.x, self.y, self.z, self.rgba) end 
ffi.metatype("PointXYZRGBA", PointXYZRGBA)

return pcl