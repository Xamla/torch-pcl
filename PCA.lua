local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local PCA = torch.class('pcl.PCA', pcl)

local func_by_type = {}

local function init()

  local PCA_method_names = {
    'new',
    'clone',
    'delete',
    'setInputCloud',
    'setIndices',
    'getMean',
    'getEigenVectors',
    'getEigenValues',
    'getCoefficients',
    'projectCloud',
    'reconstructCloud'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_PCA_TYPE_KEY_", PCA_method_names, v)
  end    
end

init()

function pcl.PointCloud:pca()
  return pcl.PCA(self)
end

function PCA:__init(pointType, basis_only)
  local cloud
  if torch.isTypeOf(pointType, pcl.PointCloud) then
    cloud = pointType
    pointType = cloud.pointType
  end
  
  pointType = pcl.pointType(pointType)
  basis_only = basis_only or false
  self.pointType = pointType
  self.f = func_by_type[self.pointType]
  
  if type(basis_only) == 'cdata' then   -- used by clone
    self.o = basis_only
  else
    self.o = self.f.new(basis_only)
  end

  if cloud then
    self:setInputCloud(cloud)
  end
end

function PCA:cdata()
  return self.o
end

function PCA:clone()
  local clone = self.f.clone(self.o)
  return PCA.new(self.pointType, clone)
end

function PCA:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function PCA:setIndices(indices)
  self.f.setIndices(self.o, indices:cdata())
end

function PCA:getMean()
  local t = torch.FloatTensor()
  self.f.getMean(self.o, t:cdata())
  return t;
end

function PCA:getEigenVectors()
  local t = torch.FloatTensor()
  self.f.getEigenVectors(self.o, t:cdata())
  return t;
end

function PCA:getEigenValues()
  local t = torch.FloatTensor()
  self.f.getEigenValues(self.o, t:cdata())
  return t;
end

function PCA:getCoefficients()
  local t = torch.FloatTensor()
  self.f.getCoefficients(self.o, t:cdata())
  return t;
end

function PCA:project(input, output)
  output = output or input
  self.f.projectCloud(self.o, input:cdata(), output:cdata())
  return output
end

function PCA:reconstruct(input, output)
  output = output or input
  self.f.reconstructCloud(self.o, input:cdata(), output:cdata())
  return output
end
