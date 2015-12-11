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
    'set_inputCloud',
    'get_mean',
    'get_eigenVectors',
    'get_eigenValues',
    'get_coefficients',
    'project_cloud',
    'reconstruct_cloud'
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
  
  pointType = pointType or pcl.PointXYZ
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
  self.f.set_inputCloud(self.o, cloud:cdata())
end

function PCA:getMean()
  local t = torch.FloatTensor()
  self.f.get_mean(self.o, t:cdata())
  return t;
end

function PCA:getEigenVectors()
  local t = torch.FloatTensor()
  self.f.get_eigenVectors(self.o, t:cdata())
  return t;
end

function PCA:getEigenValues()
  local t = torch.FloatTensor()
  self.f.get_eigenValues(self.o, t:cdata())
  return t;
end

function PCA:getCoefficients()
  local t = torch.FloatTensor()
  self.f.get_coefficients(self.o, t:cdata())
  return t;
end

function PCA:project(input, output)
  output = output or input
  self.f.project_cloud(self.o, input:cdata(), output:cdata())
  return output
end

function PCA:reconstruct(input, output)
  output = output or input
  self.f.reconstruct_cloud(self.o, input:cdata(), output:cdata())
  return output
end
