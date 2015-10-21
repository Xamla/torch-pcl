require 'torch'
local ffi = require 'ffi'

local ALIGN16 = "__attribute__((aligned(16)))"
local PCL_POINT4D = "union ".. ALIGN16.." { float data[4]; struct { float x; float y; float z; }; };"
local PCL_NORMAL4D = "union "..ALIGN16.." { float data_n[4]; float normal[3]; struct { float normal_x; float normal_y; float normal_z; }; };"
local PCL_RGB = "union { union { struct { uint8_t b; uint8_t g; uint8_t r; uint8_t a; }; uint32_t rgba; }; uint32_t rgba; };"

local cdef = "typedef struct RGB { "..PCL_RGB.." } RGB; \z
typedef struct PointXY { float x; float y; } PointUV; \z
typedef struct PointUV { float u; float v; } PointUV; \z
typedef struct "..ALIGN16.." InterestPoint { "..PCL_POINT4D.." union { struct { float strength; }; float data_c[4]; }; } InterestPoint; \z
typedef struct "..ALIGN16.." _PointXYZ { "..PCL_POINT4D.."} _PointXYZ; \z
typedef struct "..ALIGN16.." _PointXYZI { "..PCL_POINT4D.." union { struct { float intensity; }; float data_c[4]; }; } _PointXYZI; \z
typedef struct "..ALIGN16.." _PointXYZL { "..PCL_POINT4D.." uint32_t label; } _PointXYZL; \z
typedef struct "..ALIGN16.." _PointXYZRGBA { "..PCL_POINT4D..PCL_RGB.." } _PointXYZRGBA; \z
typedef struct "..ALIGN16.." _PointXYZRGBL { "..PCL_POINT4D..PCL_RGB.." uint32_t label; } _PointXYZRGBL; \z
typedef struct "..ALIGN16.." _Normal { "..PCL_POINT4D.." union { struct { float curvature; }; float data_c[4]; }; } _Normal; \z
typedef struct "..ALIGN16.." _Axis { "..PCL_NORMAL4D.." } _Axis; \z
typedef struct "..ALIGN16.." _PointNormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float curvature; }; float data_c[4]; }; } _PointNormal; \z
typedef struct "..ALIGN16.." _PointXYZRGBNormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { "..PCL_RGB.." float curvature; }; float data_c[4]; }; } _PointXYZRGBNormal; \z
typedef struct "..ALIGN16.." _PointXYZINormal { "..PCL_POINT4D..PCL_NORMAL4D.." union { struct { float intensity; float curvature; }; float data_c[4]; }; } _PointXYZINormal;" ..
[[
void* pcl_PointCloud_XYZ_new(uint32_t width, uint32_t height);
void pcl_PointCloud_XYZ_delete(void* self);
bool pcl_PointCloud_XYZ_empty(void* self);
void pcl_PointCloud_XYZ_clear(void* self);
_PointXYZ& pcl_PointCloud_at1D(void* self, int n);
void pcl_PointCloud_XYZ_totensor(void* self, void* destination);
THFloatStorage* pcl_PointCloud_XYZ_storage(void* self);
]]

ffi.cdef(cdef)

local p = ffi.load('build/debug/libtorch-pcl.so')


local PointCloud = {}

local types = {
    "XYZ",            -- float x, y, z;    
    "XYZI",           -- float x, y, z, intensity;
    "XYZRGBA",        -- float x, y, z; uint32_t rgba;
    "XYZRGB",         -- float x, y, z, rgb;
    "XY",             -- float x, y;
    "Normal",         -- float normal[3], curvature;
    "XYZNormal",      -- float x, y, z; float normal[3], curvature;
    "XYZRGBNormal",   -- float x, y, z, rgb, normal[3], curvature;
    "XYZINormal"      -- float x, y, z, intensity, normal[3], curvature;
}

function PointCloud:__tostring__()
    return "PointCloud"
end


local obj = p.pcl_PointCloud_XYZ_new(10, 10)
print(torch.typename(obj))
ffi.gc(obj, p.pcl_PointCloud_XYZ_delete)

local x =p.pcl_PointCloud_at1D(obj, 3)
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


return pcl