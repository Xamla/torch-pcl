local ffi = require 'ffi'

ffi.cdef[[

typedef unsigned int		uint32_t;

void* pcl_PointCloud_XYZ_new(uint32_t width, uint32_t height);
void pcl_PointCloud_XYZ_delete(void* self);
bool pcl_PointCloud_XYZ_empty(void* self);
void pcl_PointCloud_XYZ_clear(void* self);
void pcl_PointCloud_XYZ_totensor(void* self, void* destination);
THFloatStorage* pcl_PointCloud_XYZ_storage(void* self);
]]

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
ffi.gc(obj, p.pcl_PointCloud_XYZ_delete)

local  a = p.pcl_PointCloud_XYZ_empty(obj)
print(a)
--p.pcl_PointCloud_XYZ_clear(obj)
local  b = p.pcl_PointCloud_XYZ_empty(obj)
print(b)


local s = p.pcl_PointCloud_XYZ_storage(obj)
print(s)
local t =torch.FloatTensor()
t:cdata().storage = t
print(t)
local s2 = t:storage()
print(s2)


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