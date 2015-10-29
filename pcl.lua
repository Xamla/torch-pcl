require 'torch'
local ffi = require 'ffi'

local PCL_POINT4D = "union __attribute__((aligned(16))) { float data[4]; struct { float x; float y; float z; }; };"
local PCL_NORMAL4D = "union __attribute__((aligned(16))) { float data_n[4]; float normal[3]; struct { float normal_x; float normal_y; float normal_z; }; };"
local PCL_RGB = "union { union { struct { uint8_t b; uint8_t g; uint8_t r; uint8_t a; }; float rgb; }; uint32_t rgba; };"

local cdef = "enum NormType { L1, L2_SQR, L2, LINF, JM, B, SUBLINEAR, CS, DIV, PF, K, KL, HIK }; \z
typedef struct { "..PCL_RGB.." } RGB; \z
typedef struct { float x; float y; } PointXY; \z
typedef struct { float u; float v; } PointUV; \z
typedef struct { "..PCL_POINT4D.." union { struct { float strength; }; float data_c[4]; }; } InterestPoint; \z
typedef struct { "..PCL_POINT4D.."} PointXYZ; \z
typedef struct { "..PCL_POINT4D.." union { struct { float intensity; }; float data_c[4]; }; } PointXYZI; \z
typedef struct { "..PCL_POINT4D.." uint32_t label; } PointXYZL; \z
typedef struct { "..PCL_POINT4D..PCL_RGB.." } PointXYZRGBA; \z
typedef struct { "..PCL_POINT4D..PCL_RGB.." uint32_t label; } PointXYZRGBL; \z
typedef struct { "..PCL_POINT4D.." union { struct { float curvature; }; float data_c[4]; }; } Normal; \z
typedef struct { "..PCL_NORMAL4D.." } Axis; \z
typedef struct { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float curvature; }; float data_c[4]; }; } PointNormal; \z
typedef struct { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { "..PCL_RGB.." float curvature; }; float data_c[4]; }; } PointXYZRGBNormal; \z
typedef struct { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float intensity; float curvature; }; float data_c[4]; }; } PointXYZINormal; \z
typedef struct { THFloatStorage* storage; uint32_t width, height, dim; } _PointsBuffer;" ..
[[
void* pcl_CloudViewer_new(const char *window_name);
void pcl_CloudViewer_delete(void *self);

void* pcl_OpenNI2Stream_XYZRGBA_new(int max_backlog);
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

void pcl_CloudViewer_TYPE_KEY_showCloud(void *self, void *cloud, const char *cloudname);
]]

local supported_keys = { 'XYZ', 'XYZI', 'XYZRGBA' }
for i,v in ipairs(supported_keys) do
  local specialized = string.gsub(generic_declarations, 'TYPE_KEY', v)
  ffi.cdef(specialized)
end

local p = ffi.load(package.searchpath('libtorch-pcl', package.cpath))

pcl.PointXYZ            = ffi.typeof('PointXYZ')            -- float x, y, z;
pcl.PointXYZI           = ffi.typeof('PointXYZI')           -- float x, y, z, intensity;
pcl.PointXYZRGBA        = ffi.typeof('PointXYZRGBA')        -- float x, y, z; uint32_t rgba;
pcl.PointXY             = ffi.typeof('PointXY')             -- float x, y;
pcl.Normal              = ffi.typeof('Normal')              -- float normal[3], curvature;
pcl.PointNormal         = ffi.typeof('PointNormal')         -- float x, y, z; float normal[3], curvature;
pcl.PointXYZRGBNormal   = ffi.typeof('PointXYZRGBNormal')   -- float x, y, z, rgb, normal[3], curvature;
pcl.PointXYZINormal     = ffi.typeof('PointXYZINormal')     -- float x, y, z, intensity, normal[3], curvature;

-- PointXYZ metatype
local PointXYZ = {
  prototype = {
    fromtensor = function(t) 
      local p = pcl.PointXYZ() 
      for i=1,3 do p[i] = t[i] end
      return p
    end,
    totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z }) end
  },
  __len = function(self) return 3 end
}

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
    totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z, self.intensity }) end
  },
  __len = function(self) return 4 end
}

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
    totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z, self.rgb }) end
  },
  __len = function(self) return 4 end
}

function PointXYZRGBA:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZRGBA.prototype[i] end end
function PointXYZRGBA:__newindex(i, v) self.data[i-1] = v end
function PointXYZRGBA:__tostring() return string.format('{ x:%f, y:%f, z:%f, rgba:%08X }', self.x, self.y, self.z, self.rgba) end 
ffi.metatype("PointXYZRGBA", PointXYZRGBA)


local PointCloud_init = dofile('PointCloud.lua')
local CloudViewer_init = dofile('CloudViewer.lua')

local type_key_map = {}
type_key_map[pcl.PointXYZ] = 'XYZ'
type_key_map[pcl.PointXYZI] = 'XYZI'
type_key_map[pcl.PointXYZRGBA] = 'XYZRGBA'
  
PointCloud_init(pcl, ffi, p, type_key_map)
CloudViewer_init(pcl, ffi, p, type_key_map)

methods = {
  create = p["pcl_OpenNI2Stream_XYZRGBA_new"],
  delete = p["pcl_OpenNI2Stream_XYZRGBA_delete"],
  start = p["pcl_OpenNI2Stream_XYZRGBA_start"],
  stop = p["pcl_OpenNI2Stream_XYZRGBA_stop"],
  read = p["pcl_OpenNI2Stream_XYZRGBA_read"] 
}

print(methods)
a = methods.create(10);
methods.start(a)
b = methods.read(a, 1000);
print('read result:');
print(b);
methods.stop(a)


--[[
local obj = PointCloud.new(pcl.PointXYZ, torch.FloatTensor({{ 4,3,2,1 }, { 5,3,2, 1 }}))
print(obj:points())
print(obj[1])
print(obj[2])


local obj2 = PointCloud.new(pcl.PointXYZ, 3, 3)
print(obj2:points())
obj:add(obj2)
print(obj:points())
]]


-- TODO:
-- loading a point cloud file and get points as torch tensors
-- local cloud = pcl.io.loadPCDFile('')

function pcl:test()
  local pc = pcl.PointCloud(pcl.PointXYZ)
  pc:loadPCDFile('data/bunny.pcd')
  local t = pc:points()
  print(t)
end
