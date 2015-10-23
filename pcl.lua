require 'torch'
local ffi = require 'ffi'

pcl = {}

local ALIGN16 = "__attribute__((aligned(16)))"
local PCL_POINT4D = "union ".. ALIGN16.." { float data[4]; struct { float x; float y; float z; }; };"
local PCL_NORMAL4D = "union "..ALIGN16.." { float data_n[4]; float normal[3]; struct { float normal_x; float normal_y; float normal_z; }; };"
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
typedef struct { THFloatStorage* storage; uint32_t width, height, dim; } _PointsBuffer;"
ffi.cdef(cdef)

local generic_declarations = 
[[
void* pcl_PointCloud_TYPE_KEY_new(uint32_t width, uint32_t height);
void* pcl_PointCloud_TYPE_KEY_clone(void* self);
void pcl_PointCloud_TYPE_KEY_delete(void* self);
uint32_t pcl_PointCloud_TYPE_KEY_width(void* self);
uint32_t pcl_PointCloud_TYPE_KEY_height(void* self);
bool pcl_PointCloud_TYPE_KEY_isDense(void* self);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at1D(void* self, int n);
PointTYPE_KEY& pcl_PointCloud_TYPE_KEY_at2D(void* self, int column, int row);
void pcl_PointCloud_TYPE_KEY_clear(void* self);
bool pcl_PointCloud_TYPE_KEY_empty(void* self);
bool pcl_PointCloud_TYPE_KEY_isOrganized(void* self);
_PointsBuffer pcl_PointCloud_TYPE_KEY_points(void* self);
void pcl_PointCloud_TYPE_KEY_add(void* self, void* other);
THFloatStorage* pcl_PointCloud_TYPE_KEY_sensorOrigin(void* self);
THFloatStorage* pcl_PointCloud_TYPE_KEY_sensorOrientation(void* self);
]]

local supported_keys = { 'XYZ', 'XYZI', 'XYZRGBA' }
for i,v in ipairs(supported_keys) do
    local specialized = string.gsub(generic_declarations, 'TYPE_KEY', v)
    ffi.cdef(specialized)
end

local p = ffi.load('build/debug/libtorch-pcl.so')

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


local PointCloud_method_names = {
    "new",
    "clone",
    "delete",
    "width",
    "height",
    "isDense",
    "at1D",
    "at2D",
    "clear",
    "empty",
    "isOrganized",
    "points",
    "sensorOrientation",
    "sensorOrigin", 
    "add"
}

local generic_names = {}
for i,n in ipairs(PointCloud_method_names) do
    generic_names[n] = "pcl_PointCloud_TYPE_KEY_" .. n
end

function create_typed_methods(type_key)
    local map = {}
    for k,v in pairs(generic_names) do
        map[k] = p[string.gsub(v, "TYPE_KEY", type_key)]
    end
    return map
end

func_by_type = {}

local type_key_map = {}
type_key_map[pcl.PointXYZ] = 'XYZ'
type_key_map[pcl.PointXYZI] = 'XYZI'
type_key_map[pcl.PointXYZRGBA] = 'XYZRGBA'

for k,v in pairs(type_key_map) do
    func_by_type[k] = create_typed_methods(v)
end

local PointCloud = torch.class('PointCloud')

function PointCloud:__init(pointType, width, height)
    self.t = pointType
    self.f = func_by_type[pointType]
    if torch.isTensor(width) then
        local sz = width:size()
        local w,h
        if width:nDimension() == 3 then
            w, h = sz[2], sz[1]
        elseif width:nDimension() == 2 then
            w, h = sz[1], 1
        end
        self.c = self.f.new(w, h)
        ffi.gc(self.c, self.f.delete)
        self:points():copy(width)
    elseif type(width) == 'cdata' then
        self.c = width
    else
        self.c = self.f.new(width, height)
        ffi.gc(self.c, self.f.delete)
    end
end

function PointCloud:clone()
    local clone = self.f.clone(self.c)
    ffi.gc(clone, self.f.delete)
    return PointCloud.new(self.t, clone)
end

function PointCloud:__index(column, row)
    local v = PointCloud[column]
    if not v and type(column) == 'number' then
        local f = self.f
        local c = self.c
        if not row then
            v = f.at1D(c, column-1)
        else
            v = f.at2D(c, column-1, row-1)
        end
    end
    return v
end

function PointCloud:width()
    return self.f.width(self.c)
end

function PointCloud:height()
    return self.f.height(self.c)
end

function PointCloud:isDense()
    return self.f.isDense(self.c)
end

function PointCloud:clear()
    self.f.clear(self.c)
end

function PointCloud:empty()
    return self.f.empty(self.c)
end

function PointCloud:isOrganized()
    return self.f.isOrganized(self.c)
end

function PointCloud:points()
    local t = torch.FloatTensor()
    local buf = self.f.points(self.c)
    t:cdata().storage = buf.storage
    t:resize(buf.height, buf.width, buf.dim)
    return t
end

function PointCloud:sensorOrigin()
    local t = torch.FloatTensor()
    local s = self.f.sensorOrigin(self.c)
    t:cdata().storage = s
    t:resize(4)
    return t
end

function PointCloud:sensorOrientation()
    local t = torch.FloatTensor()
    local s = self.f.sensorOrientation(self.c)
    t:cdata().storage = s
    t:resize(4)
    return t
end

function PointCloud:add(other)
    self.f.add(self.c, other.c)
end

function PointCloud:__tostring()
    return string.format("PointCloud (w:%d, h:%d)", self:width(), self:height())
end


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

pcl.io = io

return pcl