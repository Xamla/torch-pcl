local ffi = require 'ffi'
local class = require 'class'
local pcl = require 'pcl.PointTypes'
local utils = require 'pcl.utils'

local CloudViewer = class('CloudViewer')

local func_by_type = {}
local ft = {}

function init()
  local CloudViewer_method_names = {
    "showCloud"
  }
  
  local generic_names = {}
  for i,n in ipairs(CloudViewer_method_names) do
    generic_names[n] = "pcl_CloudViewer_TYPE_KEY_" .. n
  end
  
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_CloudViewer_TYPE_KEY_", CloudViewer_method_names, v)
  end

  ft.new = pcl.lib["pcl_CloudViewer_new"]
  ft.delete = pcl.lib["pcl_CloudViewer_delete"]
end

init()

function CloudViewer:__init(window_name)
  self.v = ft.new(window_name)
  ffi.gc(self.v, ft.delete)
end

function CloudViewer:showCloud(cloud, name)
  local f = func_by_type[cloud.pointType];
  if f then
    f.showCloud(self.v, cloud.c, name);
  end
end
