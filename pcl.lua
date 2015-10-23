require 'torch'
local ffi = require 'ffi'

pcl = {}

local ALIGN16 = "__attribute__((aligned(16)))"
local PCL_POINT4D = "union ".. ALIGN16.." { float data[4]; struct { float x; float y; float z; }; };"
local PCL_NORMAL4D = "union "..ALIGN16.." { float data_n[4]; float normal[3]; struct { float normal_x; float normal_y; float normal_z; }; };"
local PCL_RGB = "union { union { struct { uint8_t b; uint8_t g; uint8_t r; uint8_t a; }; uint32_t rgba; }; uint32_t rgba; };"

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
typedef struct { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float intensity; float curvature; }; float data_c[4]; }; } PointXYZINormal;" ..

[[
void* pcl_PointCloud_XYZ_new(uint32_t width, uint32_t height);
void pcl_PointCloud_XYZ_delete(void* self);
bool pcl_PointCloud_XYZ_empty(void* self);
void pcl_PointCloud_XYZ_clear(void* self);
PointXYZ& pcl_PointCloud_at1D(void* self, int n);
void pcl_PointCloud_XYZ_totensor(void* self, void* destination);
THFloatStorage* pcl_PointCloud_XYZ_storage(void* self);
]]

ffi.cdef(cdef)

local p = ffi.load('build/debug/libtorch-pcl.so')

pcl.PointXYZ            = ffi.typeof('PointXYZ')            -- float x, y, z;
pcl.PointXYZI           = ffi.typeof('PointXYZI')           -- float x, y, z, intensity;
pcl.PointXYZRGBA        = ffi.typeof('PointXYZRGBA')        -- float x, y, z; uint32_t rgba;
pcl.PointXY             = ffi.typeof('PointXY')             -- float x, y;
pcl.PointNormal         = ffi.typeof('Normal')              -- float normal[3], curvature;
pcl.PointNormal         = ffi.typeof('PointNormal')         -- float x, y, z; float normal[3], curvature;
pcl.PointXYZRGBNormal   = ffi.typeof('PointXYZRGBNormal')   -- float x, y, z, rgb, normal[3], curvature;
pcl.PointXYZINormal     = ffi.typeof('PointXYZINormal')     -- float x, y, z, intensity, normal[3], curvature;

-- create metatables for point types
local PointXYZ = {
    prototype = {
        fromtensor = function(t) 
            local p = pcl.PointXYZ() 
            for i=1,3 do p[i] = t[i] end
            return p
        end,
        totensor = function(self) return torch.FloatTensor({ self.x, self.y, self.z }) end
    }
}

function PointXYZ:__index(i) if type(i) == "number" then return self.data[i-1] else return PointXYZ.prototype[i] end end
function PointXYZ:__newindex(i, v) self.data[i-1] = v end
function PointXYZ:__tostring() return string.format('{ x:%f, y:%f, z:%f }', self.x, self.y, self.z) end 

ffi.metatype("PointXYZ", PointXYZ)

func_by_type = {}

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
    "sensorOrigin"
}

local generic_names = {}
for i,n in ipairs(PointCloud_method_names) do
    generic_names[n] = "pcl_PointCloud_TYPE_KEY_" .. n
end

function create_typed_methods(type_key)
    local map = {}
    for k,v in pairs(generic_names) do
        map[k] = string.gsub(v, "TYPE_KEY", type_key)
    end
    return map
end

func_by_type[pcl.PointXYZ] = create_typed_methods("XYZ")


-- test code

a = pcl.PointXYZ.fromtensor(torch.Tensor({1,2,3}))
print(a)


local PointCloud = torch.class('PointCloud')

function PointCloud.__init(pointType, width, height)
    
end

function PointCloud:__tostring()
    return "PointCloud"
end


local obj = p.pcl_PointCloud_XYZ_new(10, 10)
print(torch.typename(obj))
ffi.gc(obj, p.pcl_PointCloud_XYZ_delete)

local x = p.pcl_PointCloud_at1D(obj, 3)
print(x)
--x.y = 1.23124;


local y =p.pcl_PointCloud_at1D(obj, 3)
print(y.y)


local s = p.pcl_PointCloud_XYZ_storage(obj)
local t = torch.FloatTensor()
t:cdata().storage = s



local s2 = t:storage()
--print(s2)
--print(s2:cdata().size)
--print(s2:cdata())
--print(s)


--local ft = torch.FloatTensor()
--p.pcl_PointCloud_XYZ_totensor(ft:cdata())

--print(ft)



-- converting a torch tensor into a point cloud type
local pt = torch.rand(3, 10)
--p.PointCloud


-- loading a point cloud file and get points as torch tensors
--local cloud = pcl.io.loadPCDFile('')

--pcl.PointCloud('XYZ')

-- allocating / deallocating objects



pcl.PointT = PointT
pcl.io = io

return pcl