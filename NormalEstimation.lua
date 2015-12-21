local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local NormalEstimation = torch.class('pcl.NormalEstimation', pcl)

local func_by_type = {}

local function init()

  local NormalEstimation_method_names = {
    'new',
    'delete',
    'setInputCloud',
    'setIndices',
    'getViewPoint',
    'setViewPoint',
    'useSensorOriginAsViewPoint',
    'setSearchMethod_Octree',
    'setSearchMethod_KdTree',
    'setKSearch', 
    'getKSearch',
    'setRadiusSearch',
    'getRadiusSearch',
    'compute'
  }
  
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_NormalEstimation_TYPE_KEY_", NormalEstimation_method_names, v)
  end   
end

init()

function NormalEstimation:__init(pointType)
  pointType = pcl.pointType(pointType or pcl.PointXYZ)
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  self.o = self.f.new()
end

function NormalEstimation:cdata()
  return self.o
end

function NormalEstimation:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function NormalEstimation:setIndices(indices)
  self.f.setIndices(self.o, indices:cdata())
end

function NormalEstimation:getViewPoint()
  local pt = torch.FloatTensor()
  self.f.getViewPoint(self.o, pt:cdata())
  return pt
end

function NormalEstimation:setViewPoint(pt)
  self.f.setViewPoint(self.o, pt:cdata())
end

function NormalEstimation:useSensorOriginAsViewPoint()
  self.f.useSensorOriginAsViewPoint()
end

function NormalEstimation:setSearchMethod(search)
  if torch.isTypeOf(search, pcl.KdTree) then
    self.f.setSearchMethod_KdTree(self.o, search:cdata())
  elseif torch.isTypeOf(search, pcl.Octree) then
    self.f.setSearchMethod_Octree(self.o, search:cdata())
  else
    error("unsupported search method")
  end
end

function NormalEstimation:setKSearch(k)
  self.f.setKSearch(self.o, k)
end

function NormalEstimation:getKSearch()
  return self.f.getKSearch()
end

function NormalEstimation:setRadiusSearch(radius)
  self.f.setRadiusSearch(self.o, radius)
end

function NormalEstimation:getRadiusSearch()
  return self.f.getRadiusSearch(self.o)
end

function NormalEstimation:compute(output)
  if not output then
    output = pcl.PointCloud(utils.getNormalTypeFor(self.pointType))
  end
  self.f.compute(self.o, output:cdata())
  return output
end
