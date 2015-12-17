local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local KdTreeFLANN = torch.class('pcl.KdTreeFLANN', pcl)

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

function KdTreeFLANN:__init(pointType, sorted)
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

function KdTreeFLANN:cdata()
  return self.c
end

function KdTreeFLANN:clone()
  local clone = self.f.clone(self.c)
  return KdTreeFLANN.new(self.pointType, clone)
end

function KdTreeFLANN:setInputCloud(cloud)
  self.f.setInputCloud(self.c, cloud:cdata())
end

function KdTreeFLANN:getEpsilon()
  return self.f.getEpsilon(self.c)
end
  
function KdTreeFLANN:setEpsilon(eps)
  self.f.setEpsilon(self.c, eps)
end

function KdTreeFLANN:setMinPts(value)
  self.f.setMinPts(self.c, value)
end

function KdTreeFLANN:getMinPts()
  return self.f.getMinPts(self.c)
end
  
function KdTreeFLANN:setSortedResults(sorted)
  self.f.setSortedResults(self.c, sorted)
end
  
function KdTreeFLANN:set(other)
  self.f.assign(self.c, other:cdata())
  return self
end
    
function KdTreeFLANN:nearestKSearch(point, k, indices, squaredDistances)
  return self.f.nearestKSearch(self.c, point, k, indices:cdata(), squaredDistances:cdata())
end

function KdTreeFLANN:radiusSearch(point, radius, indices, squaredDistances, max_nn)
  return self.f.radiusSearch(self.c, point, radius, indices:cdata(), squaredDistances:cdata(), max_nn or 0)
end
