local ffi = require 'ffi'
local class = require 'class'
local pcl = require 'pcl.PointTypes'
local utils = require 'pcl.utils'

local OpenNI2Stream = class('OpenNI2Stream')

local func_by_type = {}

function init()
  local OpenNI2Stream_method_names = { 'new', 'delete', 'start', 'stop', 'read' }
  local base_name = 'pcl_OpenNI2Stream_XYZRGBA_'
  
  local supported_types = {}
  supported_types[pcl.PointXYZRGBA] = "XYZRGBA"
  
  for k,v in pairs(supported_types) do
    func_by_type[k] = utils.create_typed_methods("pcl_OpenNI2Stream_TYPE_KEY_", OpenNI2Stream_method_names, v)
  end
end

init()

function OpenNI2Stream:__init(device_id, max_backlog)
  device_id = device_id or ''
  max_backlog = max_backlog or 30
  self.pointType = pcl.PointXYZRGBA
  self.f = func_by_type[self.pointType]
  self.o = self.f.new(device_id, max_backlog)
  ffi.gc(self.o, self.f.delete)
end

function OpenNI2Stream:start()
  self.f.start(self.o);
end

function OpenNI2Stream:stop()
  self.f.stop(self.o);
end

function OpenNI2Stream:read(timeout_milliseconds)
  timeout_milliseconds = timeout_milliseconds or 1000
  local frame = self.f.read(self.o, timeout_milliseconds)
  print(frame)
  if frame ~= nil then
    frame = pcl.PointCloud(self.pointType, frame)
  end
  return frame
end
