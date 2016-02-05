local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local SampleConsensusPrerejective = torch.class('pcl.SampleConsensusPrerejective', pcl)

local func_by_type = {}

local function init()
  local SampleConsensusPrerejective_method_names = {
    'new',
    'delete',
    'setInputSource',
    'setSourceFeatures',
    'setInputTarget',
    'setTargetFeatures',
    'setMaximumIterations',
    'getMaximumIterations',
    'setNumberOfSamples',
    'getNumberOfSamples',
    'getMaxCorrespondenceDistance',
    'setMaxCorrespondenceDistance',
    'getCorrespondenceRandomness',
    'setCorrespondenceRandomness',
    'setSimilarityThreshold',
    'getSimilarityThreshold',
    'setInlierFraction',
    'getInlierFraction',
    'getInliers',
    'getFinalTransformation',
    'getFitnessScore',
    'align'
  }
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_SampleConsensusPrerejective_TYPE_KEY_", SampleConsensusPrerejective_method_names, v)
  end
end

init()

function SampleConsensusPrerejective:__init(pointType)
  self.pointType = pcl.pointType(pointType or pcl.PointXYZ)
  self.f = func_by_type[self.pointType]
  self.o = self.f.new()
end

function SampleConsensusPrerejective:cdata()
  return self.o
end

function SampleConsensusPrerejective:setInputSource(source_cloud)
  self.f.setInputSource(self.o, source_cloud:cdata())
end

function SampleConsensusPrerejective:setSourceFeatures(source_features)
  self.f.setSourceFeatures(self.o, source_features:cdata())
end

function SampleConsensusPrerejective:setInputTarget(target_cloud)
  self.f.setInputTarget(self.o, target_cloud:cdata())
end

function SampleConsensusPrerejective:setTargetFeatures(target_features)
  self.f.setTargetFeatures(self.o, target_features:cdata())
end

function SampleConsensusPrerejective:setMaximumIterations(max_iterations)
  self.f.setMaximumIterations(self.o, max_iterations)
end

function SampleConsensusPrerejective:getMaximumIterations()
  return self.f.getMaximumIterations(self.o)
end

function SampleConsensusPrerejective:setNumberOfSamples(nr_samples)
  self.f.setNumberOfSamples(self.o, nr_samples)
end

function SampleConsensusPrerejective:getNumberOfSamples()
  return self.f.getNumberOfSamples(self.o)
end

function SampleConsensusPrerejective:getMaxCorrespondenceDistance()
  return self.f.getMaxCorrespondenceDistance(self.o)
end

function SampleConsensusPrerejective:setMaxCorrespondenceDistance(distance)
  self.f.setMaxCorrespondenceDistance(self.o, distance)
end

function SampleConsensusPrerejective:setCorrespondenceRandomness(k)
  self.f.setCorrespondenceRandomness(self.o, k)
end

function SampleConsensusPrerejective:getCorrespondenceRandomness()
  return  self.f.getCorrespondenceRandomness(self.o)
end

function SampleConsensusPrerejective:setSimilarityThreshold(similarity_threshold)
  self.f.setSimilarityThreshold(self.o, similarity_threshold)
end

function SampleConsensusPrerejective:getSimilarityThreshold()
  return  self.f.getSimilarityThreshold(self.o)
end

function SampleConsensusPrerejective:setInlierFraction(inlier_fraction)
  self.f.setInlierFraction(self.o, inlier_fraction)
end

function SampleConsensusPrerejective:getInlierFraction()
  return self.f.getInlierFraction(self.o)
end

function SampleConsensusPrerejective:getInliers(output)
  output = output or pcl.Indices()
  self.f.getInliers(self.o, output:cdata()) 
  return output
end

function SampleConsensusPrerejective:getFinalTransformation()
  local t = torch.FloatTensor()
  self.f.getFinalTransformation(self.o, t:cdata())
  return t
end

function SampleConsensusPrerejective:getFitnessScore(max_range)
  return self.f.getFitnessScore(self.o, max_range or pcl.range.double.max)
end

function SampleConsensusPrerejective:align(output, initial_guess)
  output = output or pcl.PointCloud(self.pointType)
  if initial_guess then
    initial_guess = initial_guess:cdata()
  end
  self.f.align(self.o, output:cdata(), initial_guess)
  return output
end
