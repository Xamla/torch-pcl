local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local EuclideanClusterExtraction = torch.class('pcl.EuclideanClusterExtraction', pcl)
local EuclideanClusterExtraction_func_by_type = {}

local function init()
  local EuclideanClusterExtraction_method_names = {
    'new',
    'delete',
    'EuclideanClusterExtraction_ptr',
    'setInputCloud',
    'setIndices',
    'setSearchMethod_Octree',
    'setSearchMethod_KdTree',
    'setClusterTolerance',
    'getClusterTolerance',
    'setMinClusterSize',
    'getMinClusterSize',
    'setMaxClusterSize',
    'getMaxClusterSize',
    'extract'
  }
  for k,v in pairs(utils.type_key_map) do
    EuclideanClusterExtraction_func_by_type[k] = utils.create_typed_methods("pcl_EuclideanClusterExtraction_TYPE_KEY_", EuclideanClusterExtraction_method_names, v)
  end
end

init()

function EuclideanClusterExtraction:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = EuclideanClusterExtraction_func_by_type[self.pointType]
  self.h = self.f.new()
  self.p = self.f.EuclideanClusterExtraction_ptr(self.h)
end

function EuclideanClusterExtraction:handle()
  return self.h
end

function EuclideanClusterExtraction:EuclideanClusterExtraction_ptr()
  return self.p
end

function EuclideanClusterExtraction:setInputCloud(cloud)
  self.f.setInputCloud(self.p, cloud:cdata())
end

function EuclideanClusterExtraction:setIndices(indices)
  self.f.setIndices(self.p, indices:cdata())
end

function EuclideanClusterExtraction:setSearchMethod(search)
  if torch.isTypeOf(search, pcl.KdTree) then
    self.f.setSearchMethod_KdTree(self.p, search:cdata())
  elseif torch.isTypeOf(search, pcl.Octree) then
    self.f.setSearchMethod_Octree(self.p, search:cdata())
  else
    error("unsupported search method")
  end
end

function EuclideanClusterExtraction:setClusterTolerance(cluster_tolerance)
  self.f.setClusterTolerance(self.p, cluster_tolerance)
end

function EuclideanClusterExtraction:getClusterTolerance()
  return self.f.getClusterTolerance(self.p)
end

function EuclideanClusterExtraction:setMinClusterSize(min_cluster_size)
  self.f.setMinClusterSize(self.p, min_cluster_size)
end

function EuclideanClusterExtraction:getMinClusterSize()
  return self.f.getMinClusterSize(self.p)
end

function EuclideanClusterExtraction:setMaxClusterSize(max_cluster_size)
  self.f.setMaxClusterSize(self.p, max_cluster_size)
end

function EuclideanClusterExtraction:getMaxClusterSize()
  return self.f.getMaxClusterSize(self.p)
end

function EuclideanClusterExtraction:extract(clusters)
  clusters = clusters or pcl.IndicesVector()
  self.f.extract(self.p, clusters:cdata())
  return clusters
end
