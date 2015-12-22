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
    self.o = self.f.new(sorted)
  elseif type(sorted) == 'cdata' then
    self.o = sorted
  end
end

function KdTree:cdata()
  return self.o
end

function KdTree:clone()
  local clone = self.f.clone(self.o)
  return KdTree.new(self.pointType, clone)
end

function KdTree:setInputCloud(cloud, indices)
  self.f.setInputCloud(self.o, cloud:cdata(), utils.cdata(indices))
end

function KdTree:getEpsilon()
  return self.f.getEpsilon(self.o)
end
  
function KdTree:setEpsilon(eps)
  self.f.setEpsilon(self.o, eps)
end

function KdTree:setMinPts(value)
  self.f.setMinPts(self.o, value)
end

function KdTree:getMinPts()
  return self.f.getMinPts(self.o)
end
  
function KdTree:setSortedResults(sorted)
  self.f.setSortedResults(self.o, sorted)
end
  
function KdTree:set(other)
  self.f.assign(self.o, other:cdata())
  return self
end
    
function KdTree:nearestKSearch(point, k, indices, squaredDistances)
  return self.f.nearestKSearch(self.o, point, k, utils.cdata(indices), utils.cdata(squaredDistances))
end

function KdTree:radiusSearch(point, radius, indices, squaredDistances, max_nn)
  return self.f.radiusSearch(self.o, point, radius, utils.cdata(indices), utils.cdata(squaredDistances), max_nn or 0)
end
