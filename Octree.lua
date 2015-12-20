local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local Octree = torch.class('pcl.Octree', pcl)

local func_by_type = {}

local function init()

  local OctreePointCloudSearch_method_names = {
    'new',
    'delete',
    'getResolution',
    'getEpsilon',
    'setEpsilon',
    'setInputCloud',
    'addPointsFromInputCloud',
    'addPointToCloud',
    'isVoxelOccupiedAtPoint',
    'deleteTree', 
    'setMaxVoxelIndex',
    'setTreeDepth',
    'getTreeDepth',
    'getLeafCount',
    'getBranchCount',
    'nearestKSearch',
    'radiusSearch'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_OctreePointCloudSearch_TYPE_KEY_", OctreePointCloudSearch_method_names, v)
  end    
end

init()

function Octree:__init(pointType, resolution)
  if type(pointType) == 'number' then
    resolution = pointType
    pointType = pcl.PointXYZ
  end
  
  utils.check_arg('resolution', type(resolution) == 'number', 'resolution not specified')
  
  pointType = pcl.pointType(pointType)
  rawset(self, 'f', func_by_type[pointType])
  self.o = self.f.new(resolution)
end

function Octree:cdata()
  return self.o
end

function Octree:getResolution()
  return self.f.getResolution(self.o)
end

function Octree:getEpsilon()
  return self.f.getEpsilon(self.o)
end

function Octree:setEpsilon(eps)
  self.f.setEpsilon(self.o, eps)
end

function Octree:setInputCloud(cloud)
  self.f.setInputCloud(self.o, cloud:cdata())
end

function Octree:addPointsFromInputCloud()
  self.f.addPointsFromInputCloud(self.o)
end

function Octree:addPointToCloud(point, cloud)
  self.f.addPointToCloud(self.o, point, cloud:cdata())
end

function Octree:isVoxelOccupiedAtPoint(p)
  self.f.isVoxelOccupiedAtPoint(self.o, p)
end

function Octree:deleteTree()
  self.f.deleteTree(self.o)
end

function Octree:setMaxVoxelIndex(max_index)
  self.f.setMaxVoxelIndex(self.o, max_index)
end

function Octree:setTreeDepth(depth)
  return self.f.setTreeDepth(self.o, depth)
end

function Octree:getTreeDepth()
  return self.f.getTreeDepth(self.o)
end

function Octree:getLeafCount()
  return self.f.getLeafCount(self.o)
end

function Octree:getBranchCount()
  return self.f.getBranchCount(self.o)
end

function Octree:nearestKSearch(point, k, indices, squaredDistances)
  return self.f.nearestKSearch(self.o, point, k, utils.opt(indices), utils.opt(squaredDistances))
end

function Octree:radiusSearch(point, radius, indices, squaredDistances, max_nn)
  return self.f.radiusSearch(self.o, point, radius, utils.opt(indices), utils.opt(squaredDistances), max_nn or 0)
end
