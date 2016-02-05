local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local VFHEstimation = torch.class('pcl.VFHEstimation', pcl)

local func_by_type = {}

local function init()

  local VFHEstimation_method_names = {
    'new',
    'delete',
    'setInputCloud',
    'setInputNormals',
    'setIndices',
    'getViewPoint',
    'setViewPoint',
    'setUseGivenNormal',
    'setNormalToUse',
    'setUseGivenCentroid',
    'setCentroidToUse',
    'setNormalizeBins',
    'setNormalizeDistance',
    'setFillSizeComponent',
    'compute',
    'setKSearch',
    'getKSearch',
    'setRadiusSearch',
    'getRadiusSearch'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_VFHEstimation_TYPE_KEY_", VFHEstimation_method_names, v)
  end
end

init()

function VFHEstimation:__init(pointType)
  pointType = pcl.pointType(pointType or pcl.PointXYZ)
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  self.o = self.f.new()
end

function VFHEstimation:cdata()
  return self.o
end

function VFHEstimation:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function VFHEstimation:setInputNormals(normals)
  self.f.setInputNormals(self.o, normals:cdata())
end

function VFHEstimation:setIndices(indices)
  self.f.setIndices(self.o, indices:cdata())
end

function VFHEstimation:getViewPoint()
  local pt = torch.FloatTensor()
  self.f.getViewPoint(self.o, pt:cdata())
  return pt
end

function VFHEstimation:setViewPoint(pt)
  self.f.setViewPoint(self.o, pt:cdata())
end
function VFHEstimation:setUseGivenNormal(use)
  self.f.setUseGivenNormal(self.o, use)
end

function VFHEstimation:setNormalToUse(normal)
  self.f.setNormalToUse(self.o, normal:cdata())
end

function VFHEstimation:setUseGivenCentroid(use)
  self.f.setUseGivenCentroid(self.o, use)
end

function VFHEstimation:setCentroidToUse(centroid)
  self.f.setCentroidToUse(self.o, centroid:cdata())
end

function VFHEstimation:setNormalizeBins(normalize)
  self.f.setNormalizeBins(self.o, normalize)
end

function VFHEstimation:setNormalizeDistance(normalize)
  self.f.setNormalizeDistance(self.o, normalize)
end

function VFHEstimation:setFillSizeComponent(fill_size)
  self.f.setFillSizeComponent(self.o, fill_size)
end

function VFHEstimation:compute(output)
  if not output then
    output = pcl.PointCloud(pcl.VFHSignature308)
  end
  self.f.compute(self.o, output:cdata())
  return output
end

function VFHEstimation:setNumberOfThreads(num_threads)
  self.f.setNumberOfThreads(self.o, num_threads)
end

function VFHEstimation:setKSearch(k)
  self.f.setKSearch(self.o, k)
end

function VFHEstimation:getKSearch()
  self.f.getKSearch(self.o)
end

function VFHEstimation:setRadiusSearch(radius)
  self.f.setRadiusSearch(self.o, radius)
end

function VFHEstimation:getRadiusSearch()
  self.f.getRadiusSearch(self.o)
end
