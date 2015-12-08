local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local ICP = torch.class('pcl.ICP', pcl)

local func_by_type = {}

local function init()

  local ICP_method_names = {
    'new',
    'delete',
    'setInputSource',
    'setInputTarget',
    'setMaxCorrespondenceDistance',
    'setMaximumIterations',
    'setTransformationEpsilon',
    'setEuclideanFitnessEpsilon',
    'getFinalTransformation',
    'align'
  }

  local supported_types = {}
  supported_types[pcl.PointXYZ] = 'XYZ'
  supported_types[pcl.PointXYZI] = 'XYZI'
  supported_types[pcl.PointXYZRGBA] = 'XYZRGBA'
  
  for k,v in pairs(supported_types) do
    func_by_type[k] = utils.create_typed_methods("pcl_ICP_TYPE_KEY_", ICP_method_names, v)
  end    
end

init()

function ICP:__init(pointType)
  local cloud
  if torch.isTypeOf(pointType, pcl.PointCloud) then
    cloud = pointType
    pointType = cloud.pointType
  end
  
  pointType = pointType or pcl.PointXYZ
  
  self.pointType = pointType
  self.f = func_by_type[self.pointType]
  self.o = self.f.new()
end

function ICP:cdata()
  return self.o
end

function ICP:setInputSource(cloud)
  self.f.setInputSource(self.o, cloud:cdata())
end

function ICP:setInputTarget(cloud)
  self.f.setInputTarget(self.o, cloud:cdata())
end

function ICP:setMaxCorrespondenceDistance(distance)
  self.f.setMaxCorrespondenceDistance(self.o, distance)
end

function ICP:setMaximumIterations(count)
  self.f.setMaximumIterations(self.o, count)
end

function ICP:setTransformationEpsilon(epsilon)
  self.f.setTransformationEpsilon(self.o, epsilon)
end

function ICP:setEuclideanFitnessEpsilon(epsilon)
  self.f.setEuclideanFitnessEpsilon(self.o, epsilon)
end

function ICP:getFinalTransformation()
  local t = torch.FloatTensor()
  self.f.getFinalTransformation(self.o, t:cdata())
  return t
end

function ICP:align(output)
  output = output or pcl.PointCloud(self.pointType)
  self.f.align(self.o, output:cdata())
  return output
end
