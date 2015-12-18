local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local KdTree = torch.class('pcl.KdTree', pcl)

local func_by_type = {}

local function init()

  local KdTreeFLANN_method_names = {
    'new',
    'clone',
    'delete',
    'setInputCloud',
    'getEpsilon',
    'setEpsilon',
    'setMinPts',
    'getMinPts',
    'setSortedResults',
    'assign',
    'nearestKSearch', 
    'radiusSearch'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_KdTreeFLANN_TYPE_KEY_", KdTreeFLANN_method_names, v)
  end    
end

init()

function KdTree:__init(pointType, sorted)
  if type(pointType) == 'boolean' then
    sorted = pointType
    pointType = pcl.PointXYZ
  end
  sorted = sorted or true
  pointType = pcl.pointType(pointType)
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  if type(sorted) == 'boolean' then
    self.c = self.f.new(sorted)
  elseif type(sorted) == 'cdata' then
    self.c = sorted
  end
end

function KdTree:cdata()
  return self.c
end

function KdTree:clone()
  local clone = self.f.clone(self.c)
  return KdTree.new(self.pointType, clone)
end

function KdTree:setInputCloud(cloud)
  self.f.setInputCloud(self.c, cloud:cdata())
end

function KdTree:getEpsilon()
  return self.f.getEpsilon(self.c)
end
  
function KdTree:setEpsilon(eps)
  self.f.setEpsilon(self.c, eps)
end

function KdTree:setMinPts(value)
  self.f.setMinPts(self.c, value)
end

function KdTree:getMinPts()
  return self.f.getMinPts(self.c)
end
  
function KdTree:setSortedResults(sorted)
  self.f.setSortedResults(self.c, sorted)
end
  
function KdTree:set(other)
  self.f.assign(self.c, other:cdata())
  return self
end
    
function KdTree:nearestKSearch(point, k, indices, squaredDistances)
  return self.f.nearestKSearch(self.c, point, k, indices:cdata(), squaredDistances:cdata())
end

function KdTree:radiusSearch(point, radius, indices, squaredDistances, max_nn)
  return self.f.radiusSearch(self.c, point, radius, indices:cdata(), squaredDistances:cdata(), max_nn or 0)
end
