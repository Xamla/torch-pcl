require 'torch'
require 'paths'
local ffi = require 'ffi'

local pcl = require 'pcl.PointTypes'
require 'pcl.PointCloud'
require 'pcl.CloudViewer'
require 'pcl.OpenNI2Stream'
require 'pcl.PCA'
require 'pcl.ICP'
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

function pcl.transformCloud(source, destination, transform)
  if not pcl.isPointCloud(source) then
    error('point cloud expected')
  end
  return source:transform(transform, destination)
end

function pcl.getMinMax3D(cloud)
  return cloud:getMinMax3D()
end

return pcl