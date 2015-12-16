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

