local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local OrganizedEdgeBase = torch.class('pcl.OrganizedEdgeBase', pcl)
local OrganizedEdgeFromNormals = torch.class('pcl.OrganizedEdgeFromNormals', 'pcl.OrganizedEdgeBase', pcl)

local OrganizedEdgeBase_func_by_type = {}
local OrganizedEdgeFromNormals_func_by_type = {}

local function init()
  local OrganizedEdgeBase_method_names = {
    'new',
    'delete',
    'OrganizedEdgeBase_ptr',
    'setInputCloud',
    'setDepthDisconThreshold',
    'setMaxSearchNeighbors',
    'setEdgeType',
    'compute'
  }
  for k,v in pairs(utils.type_key_map) do
    OrganizedEdgeBase_func_by_type[k] = utils.create_typed_methods("pcl_OrganizedEdgeBase_TYPE_KEY_", OrganizedEdgeBase_method_names, v)
  end

  local OrganizedEdgeFromNormals_method_names = {
    'new',
    'delete',
    'OrganizedEdgeFromNormals_ptr',
    'OrganizedEdgeBase_ptr',
    'setInputNormals',
    'setHCCannyLowThreshold',
    'setHCCannyHighThreshold',
    'compute'
  }
  for k,v in pairs(utils.type_key_map) do
    OrganizedEdgeFromNormals_func_by_type[k] = utils.create_typed_methods("pcl_OrganizedEdgeFromNormals_TYPE_KEY_", OrganizedEdgeFromNormals_method_names, v)
  end
end

init()

pcl.EDGELABEL =
{
  NAN_BOUNDARY      = 1,
  OCCLUDING         = 2,
  OCCLUDED          = 4,
  HIGH_CURVATURE    = 8,
  RGB_CANNY         = 16
}

function OrganizedEdgeBase:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = OrganizedEdgeBase_func_by_type[self.pointType]
  self.h = self.f.new()
  self.p = self.f.OrganizedEdgeBase_ptr(self.h)
end

function OrganizedEdgeBase:handle()
  return self.h
end

function OrganizedEdgeBase:OrganizedEdgeBase_ptr()
  return self.p
end

function OrganizedEdgeBase:setInputCloud(cloud)
  self.f.setInputCloud(self.p, cloud:cdata())
end

function OrganizedEdgeBase:setDepthDisconThreshold(th)
  self.f.setDepthDisconThreshold(self.p, th)
end

function OrganizedEdgeBase:setMaxSearchNeighbors(th)
  self.f.setMaxSearchNeighbors(self.p, th)
end

function OrganizedEdgeBase:setEdgeType(edge_types)
  self.f.setEdgeType(self.p, edge_types)
end

function OrganizedEdgeBase:compute(labels, label_indices)
  labels = labels or pcl.PointCloud(pcl.Label)
  label_indices = label_indices or pcl.IndicesVector()
  self.f.compute(self.p, labels:cdata(), label_indices:cdata())
  return labels, label_indices
end

--[[
  OrganizedEdgeFromNormals
]]
function OrganizedEdgeFromNormals:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = OrganizedEdgeBase_func_by_type[self.pointType]
  self.f2 = OrganizedEdgeFromNormals_func_by_type[self.pointType]
  self.h = self.f2.new()
  self.p = self.f2.OrganizedEdgeBase_ptr(self.h)
  self.pn = self.f2.OrganizedEdgeFromNormals_ptr(self.h)
end

function OrganizedEdgeFromNormals:handle()
  return self.h
end

function OrganizedEdgeFromNormals:SACSegmentationFromNormals_ptr()
  return self.pn
end

function OrganizedEdgeFromNormals:setInputNormals(normals)
  self.f2.setInputNormals(self.pn, normals:cdata())
end

function OrganizedEdgeFromNormals:setHCCannyLowThreshold(th)
  self.f2.setHCCannyLowThreshold(self.pn, th)
end

function OrganizedEdgeFromNormals:setHCCannyHighThreshold(th)
  self.f2.setHCCannyHighThreshold(self.pn, th)
end
  
function OrganizedEdgeFromNormals:compute(labels, label_indices)
  labels = labels or pcl.PointCloud(pcl.Label)
  label_indices = label_indices or pcl.IndicesVector()
  self.f2.compute(self.pn, labels:cdata(), label_indices:cdata())
  return labels, label_indices
end
