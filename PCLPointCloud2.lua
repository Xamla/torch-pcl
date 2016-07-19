local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local PCLPointCloud2 = torch.class('pcl.PCLPointCloud2', pcl)

local func = {}
local field_names_type_map = {}

local function init()
  local PCLPointCloud2_method_names = {
    'new',
    'delete',
    'getFieldNames',
    'tostring'
  }

  func = utils.create_typed_methods("pcl_PCLPointCloud2_", PCLPointCloud2_method_names, '')

  field_names_type_map['xyz'] = pcl.PointXYZ
  field_names_type_map['xyzintensity'] = pcl.PointXYZI
  field_names_type_map['xyzrgba'] = pcl.PointXYZRGBA
  field_names_type_map['xyznormal_xnormal_ynormal_zcurvature'] = pcl.PointNormal
  field_names_type_map['xyzintensitynormal_xnormal_ynormal_zcurvature'] = pcl.PointXYZINormal
  field_names_type_map['xyzrgbnormal_xnormal_ynormal_zcurvature'] = pcl.PointXYZRGBNormal
end

init()

function PCLPointCloud2:__init(name, create_interactor)
  rawset(self, 'f', func)
  self.o = self.f.new()
end

function PCLPointCloud2:cdata()
  return self.o
end

function PCLPointCloud2:getFieldNames()
  local string_buffer = torch.ByteStorage()
  self.f.getFieldNames(self.o, string_buffer:cdata())
  return string_buffer:string()
end

function PCLPointCloud2:toPointCloud(pointType)
  local fieldNames = self:getFieldNames()
  pointType = pointType or field_names_type_map[fieldNames]
  if pointType == nil then
    error('Cannot automatically convert point cloud: Unknown field type combination. Please explicitly specify point type.')
  end
  local cloud = pcl.PointCloud(pointType)
  cloud:fromPCLPointCloud2(self)
  return cloud
end

function PCLPointCloud2:__tostring()
  local string_buffer = torch.ByteStorage()
  self.f.tostring(self.o, string_buffer:cdata())
  return string_buffer:string()
end
