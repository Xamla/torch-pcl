local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local ICPNL = torch.class('pcl.ICPNL', pcl)

local func_by_type = {}

local function init()

  local ICPNL_method_names = {
    'new',
    'delete',
    'setInputSource',
    'setInputTarget',
    'setMaxCorrespondenceDistance',
    'setMaximumIterations',
    'setTransformationEpsilon',
    'setEuclideanFitnessEpsilon',
    'getFinalTransformation',
    'getFitnessScore', 
    'align',
    'addDistanceRejector',
    'addSurfaceNormalRejector',
    'addRANSACRejector',
    'addOneToOneRejector',
    'addTrimmedRejector'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_ICPNL_TYPE_KEY_", ICPNL_method_names, v)
  end    
end

init()

function ICPNL:__init(pointType)
  local cloud
  if torch.isTypeOf(pointType, pcl.PointCloud) then
    cloud = pointType
    pointType = cloud.pointType
  end
  
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = func_by_type[self.pointType]
  self.o = self.f.new()
end

function ICPNL:cdata()
  return self.o
end

function ICPNL:setInputSource(cloud)
  self.f.setInputSource(self.o, cloud:cdata())
end

function ICPNL:setInputTarget(cloud)
  self.f.setInputTarget(self.o, cloud:cdata())
end

function ICPNL:setMaxCorrespondenceDistance(distance)
  self.f.setMaxCorrespondenceDistance(self.o, distance)
end

function ICPNL:setMaximumIterations(count)
  self.f.setMaximumIterations(self.o, count)
end

function ICPNL:setTransformationEpsilon(epsilon)
  self.f.setTransformationEpsilon(self.o, epsilon)
end

function ICPNL:setEuclideanFitnessEpsilon(epsilon)
  self.f.setEuclideanFitnessEpsilon(self.o, epsilon)
end

function ICPNL:getFinalTransformation()
  local t = torch.FloatTensor()
  self.f.getFinalTransformation(self.o, t:cdata())
  return t
end

function ICPNL:getFitnessScore(max_range)
  return self.f.getFitnessScore(self.o, max_range or pcl.range.double.max)
end

function ICPNL:align(output, initial_guess)
  output = output or pcl.PointCloud(self.pointType)
  if initial_guess then
    initial_guess = initial_guess:cdata()
  end
  self.f.align(self.o, output:cdata(), initial_guess)
  return output
end

function ICPNL:addDistanceRejector(max_distance)
  self.f.addDistanceRejector(self.o, max_distance or 0.05)
end

function ICPNL:addSurfaceNormalRejector(threshold)
  self.f.addSurfaceNormalRejector(self.o, threshold or 1)
end

function ICPNL:addRANSACRejector(inlier_threshold, max_iterations)
  self.f.addRANSACRejector(self.o, inlier_threshold or 0.05, max_iterations or 1000)
end

function ICPNL:addOneToOneRejector()
  self.f.addOneToOneRejector(self.o)
end

function ICPNL:addTrimmedRejector()
  self.f.addTrimmedRejector(self.o, overlap_ratio or 0.5, min_correspondences or 0)
end