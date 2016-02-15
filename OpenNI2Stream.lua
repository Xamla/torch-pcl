local ffi = require 'ffi'
local torch = require 'torch'
local pcl = require 'pcl.PointTypes'
local utils = require 'pcl.utils'

local OpenNI2Stream = torch.class('pcl.OpenNI2Stream', pcl)

local func_by_type = {}

local function init()
  local OpenNI2Stream_method_names = { 'new', 'delete', 'start', 'stop', 'read', 'readRGBImage', 'readDepthImage', 'readIRImage' }
  local base_name = 'pcl_OpenNI2Stream_XYZRGB_'
  
  local supported_types = {}
  supported_types[pcl.PointXYZ] = "XYZ"
  supported_types[pcl.PointXYZI] = "XYZI"
  supported_types[pcl.PointXYZRGBA] = "XYZRGBA" 
  
  for k,v in pairs(supported_types) do
    func_by_type[k] = utils.create_typed_methods("pcl_OpenNI2Stream_TYPE_KEY_", OpenNI2Stream_method_names, v)
  end
end

init()

function OpenNI2Stream:__init(pointType, device_id, max_backlog, grab_RGB, grab_depth, grab_IR)
  device_id = device_id or ''
  max_backlog = max_backlog or 2
  self.pointType = pcl.pointType(pointType)
  self.f = func_by_type[self.pointType]
  self.o = self.f.new(device_id, max_backlog, grab_RGB or false, grab_depth or false, grab_IR or false)
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
  if frame ~= nil then
    frame = pcl.PointCloud(self.pointType, frame)
    ffi.gc(frame:cdata(), rawget(frame, 'f').delete)
  end
  return frame
end

function OpenNI2Stream:readRGBImage(timeout_milliseconds, result)
  result = result or torch.ByteTensor()
  self.f.readRGBImage(self.o, timeout_milliseconds or 1000, result:cdata())
  return result
end

function OpenNI2Stream:readDepthImage(timeout_milliseconds, result)
  result = result or torch.ShortTensor()
  self.f.readDepthImage(self.o, timeout_milliseconds or 1000, result:cdata())
  return result
end

function OpenNI2Stream:readIRImage(timeout_milliseconds, result)
  result = result or torch.ShortTensor()
  self.f.readIRImage(self.o, timeout_milliseconds or 1000, result:cdata())
  return result
end
