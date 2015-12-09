local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local filter = {}
pcl.filter = filter

local func_by_type = {}

local function init()

  local method_names = {
    'passThrough',
    'cropBox',
    'voxelGrid',
    'statisticalOutlierRemoval',
    'randomSample',
    'medianFilter',
    'radiusOutlierRemoval'
  }

  local supported_types = {}
  supported_types[pcl.PointXYZ] = 'XYZ'
  supported_types[pcl.PointXYZI] = 'XYZI'
  supported_types[pcl.PointXYZRGBA] = 'XYZRGBA'
  
  for k,v in pairs(supported_types) do
    func_by_type[k] = utils.create_typed_methods("pcl_Filter_TYPE_KEY_", method_names, v)
  end    
end

init()

local function check_arg(argName, check, errorMsg)
  if not check then
    error("Invalid argument '" .. argName .. " ': " .. errorMsg)
  end
end

local function check_inout_type(input, output)
  check_arg('input', pcl.isPointCloud(input), 'point cloud expected')
  check_arg('output', pcl.isPointCloud(output), 'point cloud expected')
  
  if input.pointType != outptu.pointType
  
  return func_by_type[input.pointType]
end

function filter.passThrough(input, output, fieldName, min, max, negative)
  output = output or input
  local f = check_inout_type(input, output)
  
  if type(fieldName) ~= 'string' then
    error('Invalid argument fieldName: string expected')
  end
  min = min or pcl.range.double.min
  max = max or pcl.range.double.max
  f.passThrough(input:cdata(), output:cdata(), fieldName, min, max, negative or false);
  return output
end

function filter.cropBox(input, output, min, max, rotation, translation)
  output = output or input
  local f = check_inout_type(input, output)
  
  f.cropBox(input:cdata(), output:cdata(), min, max, rotation, translation);
  return output
end

function filter.voxelGrid(input, output, lx, ly, lz)
  output = output or input
  local f = check_inout_type(input, output)
  lx = lx or 1
  ly = ly or lx
  lz = lz or lx
  f.voxelGrid(input:cdata(), output:cdata(), lx, ly, lz)
  return output
end

function filter.statisticalOutlierRemoval(input, output, meanK, stddevMulThresh, negative)
  output = output or input
  local f = check_inout_type(input, output)
  f.statisticalOutlierRemoval(input:cdata(), output:cdata(), meanK, stddevMulThresh or 1, negative or false)
  return output
end

function filter.randomSample(input, output, count)
  output = output or input
  local f = check_inout_type(input, output)
  f.randomSample(input:cdata(), output:cdata(), count or pcl.range.uint32.max)
  return output
end

function filter.medianFilter(input, output, windowSize)
  output = output or input
  local f = check_inout_type(input, output)
  f.medianFilter(input:cdata(), output:cdata(), windowSize or 5)
  return output
end

function filter.radiusOutlierRemoval(input, output, radius, minNeighbors)
  output = output or input
  local f = check_inout_type(input, output)
  f.radiusOutlierRemoval(input:cdata(), output:cdata(), radius or 0, minNeighbors or 1)
  return output
end
