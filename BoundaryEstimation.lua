local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local BoundaryEstimation = torch.class('pcl.BoundaryEstimation', pcl)

local func_by_type = {}

local function init()

  local BoundaryEstimation_method_names = {
    'new',
    'delete',
    'setInputCloud',
    'setInputNormals',
    'setIndices',
    'setAngleThreshold',
    'getAngleThreshold',
    'setSearchMethod_Octree',
    'setSearchMethod_KdTree',
    'setKSearch',
    'getKSearch',
    'setRadiusSearch',
    'getRadiusSearch',
    'compute',
    'computeIndices'
  }
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_BoundaryEstimation_TYPE_KEY_", BoundaryEstimation_method_names, v)
  end    
end

init()

function BoundaryEstimation:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = func_by_type[self.pointType]
  self.o = self.f.new()
end

function BoundaryEstimation:cdata()
  return self.o
end

function BoundaryEstimation:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function BoundaryEstimation:setInputNormals(normals)
  self.f.setInputNormals(self.o, normals:cdata())
end

function BoundaryEstimation:setIndices(indices)
  self.f.setIndices(self.o, indices:cdata())
end

function BoundaryEstimation:setAngleThreshold(angle)
  self.f.setAngleThreshold(self.o, angle)
end

function BoundaryEstimation:getAngleThreshold()
  return self.f.getAngleThreshold(self.o)
end

function BoundaryEstimation:setSearchMethod(search)
  if torch.isTypeOf(search, pcl.KdTree) then
    self.f.setSearchMethod_KdTree(self.o, search:cdata())
  elseif torch.isTypeOf(search, pcl.Octree) then
    self.f.setSearchMethod_Octree(self.o, search:cdata())
  else
    error("unsupported search method")
  end
end
    
function BoundaryEstimation:setKSearch(k)
  self.f.setKSearch(self.o, k)
end

function BoundaryEstimation:getKSearch()
  return self.f.getKSearch()
end

function BoundaryEstimation:setRadiusSearch(radius)
  self.f.setRadiusSearch(self.o, radius)
end

function BoundaryEstimation:getRadiusSearch()
  return self.f.getRadiusSearch(self.o)
end

function BoundaryEstimation:compute(output)
  if not output then
    output = pcl.PointCloud(pcl.Normal)
  end
  self.f.compute(self.o, output:cdata())
  return output
end

function BoundaryEstimation:computeIndices(output)
  output = output or pcl.Indices()
  self.f.computeIndices(self.o, output:cdata())
  return output
end
