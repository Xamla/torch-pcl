local ffi = require 'ffi'
local pcl = require 'pcl.PointTypes'

local utils = {}

local type_key_map = {}
type_key_map[pcl.PointXYZ] = 'XYZ'
type_key_map[pcl.PointXYZI] = 'XYZI'
type_key_map[pcl.PointXYZRGBA] = 'XYZRGBA'
utils.type_key_map = type_key_map

function utils.create_typed_methods(prefix, names, type_key)
  local map = {}
  for i,n in ipairs(names) do
    local full_name = string.gsub(prefix .. n, 'TYPE_KEY', type_key)
    
    -- use pcall since not all types support all functions
    local ok,v = pcall(function() return pcl.lib[full_name] end)
    if ok then
      map[n] = v
    end
  end
  
  -- check whether we have new and delete functions
  -- automatically register objects created by new with the gc 
  local _new, _delete = map.new, map.delete
  if _new and _delete then
    map.new = function(...)
      local obj = _new(...)
      ffi.gc(obj, _delete)
      return obj
    end
  end
  
  return map
end

return utils