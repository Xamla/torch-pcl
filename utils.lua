local pcl = require 'torch-pcl.PointTypes'

local utils = {}

local type_key_map = {}
type_key_map[pcl.PointXYZ] = 'XYZ'
type_key_map[pcl.PointXYZI] = 'XYZI'
type_key_map[pcl.PointXYZRGBA] = 'XYZRGBA'
utils.type_key_map = type_key_map

function utils.create_typed_methods(prefix, names, type_key)
  local map = {}
  for i,n in ipairs(names) do
    local full_name = string.gsub(prefix .. n, "TYPE_KEY", type_key)
    map[n] = pcl.lib[full_name]
  end
  return map
end

return utils