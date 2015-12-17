local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local OctreePointCloudSearch = torch.class('pcl.OctreePointCloudSearch', pcl)

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

function OctreePointCloudSearch:__init(pointType, resolution)
  if type(pointType) == 'number' then
    resolution = pointType
    pointType = pcl.PointXYZ
  end
  
  utils.check_arg('resolution', type(resolution) == 'number', 'resolution not specified')
  
  pointType = pcl.pointType(pointType)
  rawset(self, 'f', func_by_type[pointType])
  self.c = self.f.new(resolution)
end

function OctreePointCloudSearch:cdata()
  return self.c
end

function OctreePointCloudSearch:getResolution()
  return self.f.getResolution(self.c)
end

function OctreePointCloudSearch:getEpsilon()
  return self.f.getEpsilon(self.c)
end

function OctreePointCloudSearch:setEpsilon(eps)
  self.f.setEpsilon(self.c, eps)
end

function OctreePointCloudSearch:setInputCloud(cloud)
  self.f.setInputCloud(self.c, cloud:cdata())
end

function OctreePointCloudSearch:addPointsFromInputCloud()
  self.f.addPointsFromInputCloud(self.c)
end

function OctreePointCloudSearch:addPointToCloud(point, cloud)
  self.f.addPointToCloud(self.c, point, cloud:cdata())
end

function OctreePointCloudSearch:isVoxelOccupiedAtPoint(p)
  self.f.isVoxelOccupiedAtPoint(self.c, p)
end

function OctreePointCloudSearch:deleteTree()
  self.f.deleteTree(self.c)
end

function OctreePointCloudSearch:setMaxVoxelIndex(max_index)
  self.f.setMaxVoxelIndex(self.c, max_index)
end

function OctreePointCloudSearch:setTreeDepth(depth)
  return self.f.setTreeDepth(self.c, depth)
end

function OctreePointCloudSearch:getTreeDepth()
  return self.f.getTreeDepth(self.c)
end

function OctreePointCloudSearch:getLeafCount()
  return self.f.getLeafCount(self.c)
end

function OctreePointCloudSearch:getBranchCount()
  return self.f.getBranchCount(self.c)
end

function OctreePointCloudSearch:nearestKSearch(point, k, indices, squaredDistances)
  return self.f.nearestKSearch(self.c, point, k, indices:cdata(), squaredDistances:cdata())
end

function OctreePointCloudSearch:radiusSearch(point, radius, indices, squaredDistances, max_nn)
  return self.f.radiusSearch(self.c, point, radius, indices:cdata(), squaredDistances:cdata(), max_nn or 0)
end
