local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local SACSegmentation = torch.class('pcl.SACSegmentation', pcl)
local SACSegmentationFromNormals, SACSegmentation = torch.class('pcl.SACSegmentationFromNormals', 'pcl.SACSegmentation', pcl)

local SACSegmentation_func_by_type = {}
local SACSegmentationFromNormals_func_by_type = {}

local function init()
  local SACSegmentation_method_names = {
    'new',
    'delete',
    'SACSegmentation_ptr',
    'setInputCloud',
    'setIndices',
    'setModelType',
    'getModelType',
    'setDistanceThreshold',
    'getDistanceThreshold',
    'setMaxIterations',
    'getMaxIterations',
    'setProbability',
    'getProbability',
    'setOptimizeCoefficients',
    'getOptimizeCoefficients',
    'setSamplesMaxDist_KdTree',
    'setSamplesMaxDist_Octree',
    'setRadiusLimits',
    'setAxis',
    'getAxis',
    'setEpsAngle',
    'getEpsAngle',
    'segment'
  }
  for k,v in pairs(utils.type_key_map) do
    SACSegmentation_func_by_type[k] = utils.create_typed_methods("pcl_SACSegmentation_TYPE_KEY_", SACSegmentation_method_names, v)
  end
  
  local SACSegmentationFromNormals_method_names = {
    'new',
    'delete',
    'SACSegmentationFromNormals_ptr',
    'SACSegmentation_ptr'
  }
  for k,v in pairs(utils.type_key_map) do
    SACSegmentationFromNormals_func_by_type[k] = utils.create_typed_methods("pcl_SACSegmentationFromNormals_TYPE_KEY_", SACSegmentationFromNormals_method_names, v)
  end
end

init()

function SACSegmentation:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = SACSegmentation_func_by_type[self.pointType]
  self.h = self.f.new()
  self.p = self.f.SACSegmentation_ptr(self.h)
end

function SACSegmentation:handle()
  return self.h
end

function SACSegmentation:SACSegmentation_ptr()
  return self.p
end

function SACSegmentation:setInputCloud(cloud)
  self.f.setInputCloud(self.p, cloud:cdata())
end

function SACSegmentation:setIndices(indices)
  self.f.setIndices(self.p, indices:cdata())
end

function SACSegmentation:setModelType(model)
  self.f.setModelType(self.p, model)
end

function SACSegmentation:getModelType()
  return self.f.getModelType(self.p)
end

function SACSegmentation:setDistanceThreshold()
  self.f.setDistanceThreshold(self.p, threshold)
end

function SACSegmentation:getDistanceThreshold()
  return self.f.getDistanceThreshold(self.p)
end

function SACSegmentation:setMaxIterations(max_iterations)
  self.f.setMaxIterations(self.p, max_iterations)
end

function SACSegmentation:getMaxIterations()
  return self.f.getMaxIterations(self.p)
end

function SACSegmentation:setProbability(probability)
  self.f.setProbability(self.p, probability)
end

function SACSegmentation:getProbability()
  return self.f.getProbability(self.p)
end

function SACSegmentation:setOptimizeCoefficients(optimize)
  self.f.setOptimizeCoefficients(self.p, optimize)
end

function SACSegmentation:getOptimizeCoefficients()
  return self.f.getOptimizeCoefficients(self.p)
end

function SACSegmentation:setSamplesMaxDist(radius, search)
  if torch.isTypeOf(search, pcl.KdTree) then
    self.f.setSamplesMaxDist_KdTree(self.p, radius, search:cdata())
  elseif torch.isTypeOf(search, pcl.Octree) then
    self.f.setSamplesMaxDist_Octree(self.p, radius, search:cdata())
  else
    error("unsupported search method")
  end
end

function SACSegmentation:setRadiusLimits(min_radius, max_radius)
  self.f.setRadiusLimits(self.p, min_radius, max_radius)
end

function SACSegmentation:setAxis(axis)
  self.f.setAxis(self.p, axis:cdata())
end

function SACSegmentation:getAxis(result)
  result = result or torch.FloatTensor()
  self.f.getAxis(self.p, result:cdata())
  return result
end

function SACSegmentation:setEpsAngle(ea)
  self.f.setEpsAngle(self.p, ea)
end

function SACSegmentation:getEpsAngle()
  return self.f.getEpsAngle(self.p)
end

function SACSegmentation:segment(inliers, coefficients)
  inliers = inliers or pcl.Indices()
  coefficients = coefficients or torch.FloatTensor()  
  self.f.segment(self.p, inliers:cdata(), coefficients:cdata())
end

--[[
  SACSegmentationFromNormals
]]
function SACSegmentationFromNormals:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = SACSegmentation_func_by_type[self.pointType]
  self.f2 = SACSegmentationFromNormals_func_by_type[self.pointType]
  self.h = self.f2.new()
  self.p = self.f2.SACSegmentation_ptr(self.h)
  self.pn = self.f2.SACSegmentationFromNormals_ptr(self.h)
end

function SACSegmentationFromNormals:handle()
  return self.h
end

function SACSegmentationFromNormals:SACSegmentationFromNormals_ptr()
  return self.pn
end

function SACSegmentationFromNormals:setInputNormals(normals)
  self.f2.setInputNormals(self.pn, normals:cdata())
end

function SACSegmentationFromNormals:setNormalDistanceWeight(distance_weight)
  self.f2.setNormalDistanceWeight(self.pn, distance_weight)
end

function SACSegmentationFromNormals:setMinMaxOpeningAngle(min_angle, max_angle)
  self.f2.setMinMaxOpeningAngle(self.pn, min_angle, max_angle)
end

function SACSegmentationFromNormals:setDistanceFromOrigin(d)
  self.f2.setDistanceFromOrigin(self.pn, d)
end
