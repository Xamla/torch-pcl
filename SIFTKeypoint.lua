local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local SIFTKeypoint = torch.class('pcl.SIFTKeypoint', pcl)

local func_by_type = {}

local function init()

  local SIFTKeypoint_method_names = {
    'new',
    'delete',
    'setInputCloud',
    'setSearchMethod_Octree',
    'setSearchMethod_KdTree',
    'setScales',
    'setMinimumContrast',
    'compute'
  }
  
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_SIFTKeypoint_TYPE_KEY_", SIFTKeypoint_method_names, v)
  end   
end

init()

function SIFTKeypoint:__init(pointType)
  pointType = pcl.pointType(pointType or pcl.PointXYZ)
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  self.o = self.f.new()
  self:setScales(0.01, 3, 4)
  self:setMinimumContrast(0.001)
end

function SIFTKeypoint:cdata()
  return self.o
end

function SIFTKeypoint:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function SIFTKeypoint:setIndices(indices)
  self.f.setIndices(self.o, indices:cdata())
end

function SIFTKeypoint:setSearchMethod(search)
  if torch.isTypeOf(search, pcl.KdTree) then
    self.f.setSearchMethod_KdTree(self.o, search:cdata())
  elseif torch.isTypeOf(search, pcl.Octree) then
    self.f.setSearchMethod_Octree(self.o, search:cdata())
  else
    error("unsupported search method")
  end
end

function SIFTKeypoint:setScales(min_scale, octaves, scales_per_octave)
  self.f.setScales(self.o, min_scale, octaves, scales_per_octave)
end

function SIFTKeypoint:setMinimumContrast(min_contrast)
  self.f.setMinimumContrast(self.o, min_contrast)
end

function SIFTKeypoint:compute(output)
  if not output then
    output = pcl.PointCloud(pcl.Normal)
  end
  self.f.compute(self.o, output:cdata())
  return output
end
