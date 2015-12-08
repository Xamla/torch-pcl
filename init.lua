require 'torch'
require 'paths'
local ffi = require 'ffi'

local pcl = require 'pcl.PointTypes'
require 'pcl.PointCloud'
require 'pcl.CloudViewer'
require 'pcl.OpenNI2Stream'
require 'pcl.PCA'
require 'pcl.affine'
require 'pcl.primitive'

function pcl.rand(width, height, pointType)
  if pcl.isPointType(height) or type(height) == 'string' then
    pointType = height
    height = 1
  else
    height = height or 1
  end
  local pc = pcl.PointCloud(pointType or pcl.PointXYZ, width, height)
  if width > 0 and height > 0 then
    pc:points()[{{},{},{1,3}}]:rand(height, width, 3)
  end
  return pc
end

function pcl.isPointCloud(obj)
  return torch.isTypeOf(obj, pcl.PointCloud)
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

function pcl.test2()
  local s = pcl.OpenNI2Stream()
  local v = pcl.CloudViewer()
  s:start()
  for i=1,10000 do
    local c = s:read(5000)
    if c ~= nil then
      v:showCloud(c)
    end
  end
  s:stop()
end



return pcl