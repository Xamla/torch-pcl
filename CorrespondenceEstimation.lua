local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local CorrespondenceEstimation = torch.class('pcl.CorrespondenceEstimation', pcl)

local func_by_type = {}

local function init()

  local CorrespondenceEstimation_method_names = {
    'new',
    'delete',
    'setInputSource',
    'setInputTarget',
    'setIndicesSource',
    'setIndicesTarget',
    'setSearchMethodSource',
    'setSearchMethodTarget',
    'determineCorrespondences',
    'determineReciprocalCorrespondences'
  }
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_CorrespondenceEstimation_TYPE_KEY_", CorrespondenceEstimation_method_names, v)
  end
  func_by_type[pcl.FPFHSignature33] = utils.create_typed_methods("pcl_CorrespondenceEstimation_TYPE_KEY_", CorrespondenceEstimation_method_names, 'FPFHSignature33')
end

init()

function CorrespondenceEstimation:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = func_by_type[self.pointType]
  self.o = self.f.new()
end

function CorrespondenceEstimation:cdata()
  return self.o
end

function CorrespondenceEstimation:setInputSource(cloud)
  self.f.setInputSource(self.o, cloud:cdata())
end

function CorrespondenceEstimation:setInputTarget(cloud)
  self.f.setInputTarget(self.o, cloud:data())
end

function CorrespondenceEstimation:setIndicesSource(indices, no_recompute)
  self.f.setIndicesSource(self.o, indices:cdata(), no_recompute or false)
end

function CorrespondenceEstimation:setIndicesTarget(indices, no_recompute)
  self.f.setIndicesTarget(self.o, indices:cdata(), no_recompute or false)
end

function CorrespondenceEstimation:setSearchMethodSource(kdtree)
  self.f.setSearchMethodSource(self.o, kdtree:cdata())
end

function CorrespondenceEstimation:setSearchMethodTarget(kdtree)
  self.f.setSearchMethodTarget(self.o, kdtree:cdata())
end

function CorrespondenceEstimation:determineCorrespondences(correspondences, max_distance)
  correspondences = correspondences or pcl.Correspondences()
  self.f.determineCorrespondences(self.o, correspondences:cdata(), max_distance or pcl.range.double.max)
  return correspondences
end

function CorrespondenceEstimation:determineReciprocalCorrespondences(correspondences, max_distance)
  correspondences = correspondences or pcl.Correspondences()
  self.f.determineReciprocalCorrespondences(self.o, correspondences:cdata(), max_distance or pcl.range.double.max)
  return correspondences
end
