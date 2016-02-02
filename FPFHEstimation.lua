local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local FPFHEstimation = torch.class('pcl.FPFHEstimation', pcl)

local func_by_type = {}

local function init()

  local FPFHEstimation_method_names = {
    'new',
    'delete',
    'setInputCloud',
    'setIndices',
    'compute',
  }
  
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_FPFHEstimation_TYPE_KEY_", FPFHEstimation_method_names, v)
  end   
end

init()

function FPFHEstimation:__init(pointType)
  pointType = pcl.pointType(pointType or pcl.PointXYZ)
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  self.o = self.f.new()
end

function FPFHEstimation:cdata()
  return self.o
end

function FPFHEstimation:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function FPFHEstimation:setIndices(indices)
  self.f.setIndices(self.o, indices:cdata())
end

function FPFHEstimation:compute(output)
  if not output then
    output = pcl.PointCloud(pcl.FPFHSignature33)
  end
  self.f.compute(self.o, output:cdata())
  return output
end

function FPFHEstimation:setNumberOfThreads(num_threads)
  self.f.setNumberOfThreads(self.o, num_threads)
end