local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local KdTreeFLANN = torch.class('pcl.KdTreeFLANN', pcl)

local func_by_type = {}

local function init()

  local KdTreeFLANN_method_names = {
    'new',
    'clone',
    'delete',
    'setInputCloud',
    'getEpsilon',
    'setEpsilon',
    'setMinPts',
    'getMinPts',
    'setSortedResults',
    'assign',
    'nearestKSearch', 
    'radiusSearch'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_KdTreeFLANN_TYPE_KEY_", KdTreeFLANN_method_names, v)
  end    
end

init()

