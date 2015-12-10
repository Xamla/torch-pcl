require 'torch'
require 'paths'
local ffi = require 'ffi'

local pcl = require 'pcl.PointTypes'
local utils = require 'pcl.utils'
require 'pcl.PointCloud'
require 'pcl.CloudViewer'
require 'pcl.OpenNI2Stream'
require 'pcl.PCA'
require 'pcl.ICP'
require 'pcl.affine'
require 'pcl.primitive'
require 'pcl.filter'

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
  return obj and torch.isTypeOf(obj, pcl.PointCloud)
end

function pcl.isPoint(obj)
  if obj and type(obj) == 'cdata' then
    local ok,t = pcall(ffi.typeof, obj)
    return ok and pcl.isPointType(t)
  end
  return false
end

function pcl.transformCloud(source, destination, transform)
  utils.check_arg('source', pcl.isPointCloud(source), 'point cloud expected')
  return source:transform(transform, destination)
end

function pcl.getMinMax3D(cloud)
  return cloud:getMinMax3D()
end

return pcl