local pcl = require 'pcl'

-- create random tensor 
local source_tensor = torch.rand(15,4)
source_tensor[{{},4}] = 1 -- set W component to 1

-- generate point cloud with element type pcl.pointXYZ from tensor
local cloud = pcl.PointCloud('xyz', source_tensor)

--[[ 
Use 'apply()' member function to call a function for each point.
If the function returns nil no points are modified,
otherwise the source point in the point cloud is overwritten
by the function result.
]]
local function dump(cloud)
  cloud:apply(function(p,i) print(p) end)
end

-- show what we got:
print(cloud)
dump(cloud)

-- use the points() member function to get a tensor view
local b = cloud:points()    -- b is now FloatTensor
print(torch.type(b))
print(b:size())

-- we can now directly manipulate point cloud memory through the tensor
local rot_transform = pcl.affine.rotateEuler(0,0,90,true)
print('transformation for 90 degree rotation around z-axis:')
print(rot_transform)

print('Rotation result:')
b[{1,{},{}}] = b:view(#cloud,4) * rot_transform:t()

dump(cloud)

print('Normal distributed:')
--[[
Fill x,y,z components with  normal distributed random values.
(select rows 1 to 3 only to keep W component 1)
]]
b[{1,{},{1,3}}]:randn(#cloud,3)  
print(b[{{},{},4}])
dump(cloud)

print('mean: ', b:mean(2))
