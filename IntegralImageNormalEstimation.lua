local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local IntegralImageNormalEstimation = torch.class('pcl.IntegralImageNormalEstimation', pcl)

local func_by_type = {}

local function init()

  local IntegralImageNormalEstimation_method_names = {
    'new',
    'delete',
    'setInputCloud',
    'setRectSize',
    'setMaxDepthChangeFactor',
    'setBorderPolicy',
    'computePointNormal',
    'computePointNormalMirror',
    'setNormalSmoothingSize',
    'setNormalEstimationMethod',
    'setDepthDependentSmoothing',
    'getViewPoint',
    'setViewPoint',
    'useSensorOriginAsViewPoint',
    'compute'
  }
  
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_IntegralImageNormalEstimation_TYPE_KEY_", IntegralImageNormalEstimation_method_names, v)
  end   
end

init()

function IntegralImageNormalEstimation:__init(pointType)
  pointType = pcl.pointType(pointType or pcl.PointXYZ)
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  self.o = self.f.new()
end

function IntegralImageNormalEstimation:cdata()
  return self.o
end

function IntegralImageNormalEstimation:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function IntegralImageNormalEstimation:setRectSize(width, height)
  self.f.setRectSize(self.o, width, height)
end

function IntegralImageNormalEstimation:setMaxDepthChangeFactor(max_depth_change_factor)
  self.f.setMaxDepthChangeFactor(self.o, max_depth_change_factor)
end

function IntegralImageNormalEstimation:setBorderPolicy(border_policy)
  self.f.setBorderPolicy(self.o, border_policy)
end

function IntegralImageNormalEstimation:computePointNormal(pos_x, pos_y, point_index)
  return self.f.computePointNormal(self.o, pos_x, pos_y, point_index)
end

function IntegralImageNormalEstimation:computePointNormalMirror(pos_x, pos_y, point_index)
  return self.f.computePointNormalMirror(self.o, pos_x, pos_y, point_index)
end

function IntegralImageNormalEstimation:setNormalSmoothingSize(normal_smoothing_size)
  self.f.setNormalSmoothingSize(self.o, normal_smoothing_size)
end

function IntegralImageNormalEstimation:setNormalEstimationMethod(normal_estimation_method)
  self.f.setNormalEstimationMethod(self.o, normal_estimation_method)
end

function IntegralImageNormalEstimation:setDepthDependentSmoothing(use_depth_dependent_smoothing)
  self.f.setDepthDependentSmoothing(self.o, use_depth_dependent_smoothing or true)
end

function IntegralImageNormalEstimation:getViewPoint()
  local pt = torch.FloatTensor()
  self.f.getViewPoint(self.o, pt:cdata())
  return pt
end

function IntegralImageNormalEstimation:setViewPoint(pt)
  self.f.setViewPoint(self.o, pt:cdata())
end

function IntegralImageNormalEstimation:useSensorOriginAsViewPoint()
  self.f.useSensorOriginAsViewPoint(self.o)
end

function IntegralImageNormalEstimation:compute(output)
  if not output then
    output = pcl.PointCloud(pcl.Normal)
  end
  self.f.compute(self.o, output:cdata())
  return output
end
