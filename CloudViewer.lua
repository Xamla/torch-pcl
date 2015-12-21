local ffi = require 'ffi'
local torch = require 'torch'
local pcl = require 'pcl.PointTypes'
local utils = require 'pcl.utils'

local CloudViewer = torch.class('pcl.CloudViewer', pcl)

local func_by_type = {}
local ft = {}

local function init()
  local CloudViewer_method_names = {
    "showCloud"
  }
  
  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_CloudViewer_TYPE_KEY_", CloudViewer_method_names, v)
  end

  ft.new = pcl.lib.pcl_CloudViewer_new
  ft.delete = pcl.lib.pcl_CloudViewer_delete
  ft.wasStopped = pcl.lib.pcl_CloudViewer_wasStopped
end

init()

function CloudViewer:__init(window_name)
  self.v = ft.new(window_name)
  ffi.gc(self.v, ft.delete)
end

function CloudViewer:showCloud(cloud, name)
  local f = func_by_type[cloud.pointType]
  if f then
    f.showCloud(self.v, cloud:cdata(), name)
  end
end

function CloudViewer:wasStopped(millis_to_wait)
  return ft.wasStopped(self.v, millis_to_wait or 1)
end