local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local filter = {}
pcl.filter = filter

local func_by_type = {}

local function init()

  local method_names = {
    'removeNaNFromPointCloud',
    'removeNaNNormalsFromPointCloud',
    'passThrough',
    'cropBox',
    'cropSphere',
    'voxelGrid',
    'statisticalOutlierRemoval',
    'randomSample',
    'medianFilter',
    'radiusOutlierRemoval',
    'voxelHistogram'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_Filter_TYPE_KEY_", method_names, v)
  end    
end

init()

local cdata = utils.cdata

local function check_input_type(input, inplace)
  utils.check_arg('input', pcl.isPointCloud(input), 'point cloud expected')
  local output = inplace and input or pcl.PointCloud(input.pointType)
  return func_by_type[input.pointType], output
end

local function tensor(x)
  if pcl.isPoint(x) then
    x = x:totensor()
  elseif type(x) == 'table' then
    x = torch.FloatTensor(x)
  end
  return x
end

function filter.removeNaNFromPointCloud(input, indices, inplace)
  local f, output = check_input_type(input, inplace)
  indices = indices or pcl.Indices()
  f.removeNaNFromPointCloud(input:cdata(), output:cdata(), indices:cdata())
  return output
end

function filter.removeNaNNormalsFromPointCloud(input, indices, inplace)
  local f, output = check_input_type(input, inplace)
  utils.check_arg('input', f.removeNaNNormalsFromPointCloud, 'unsupported point type')
  indices = indices or pcl.Indices()
  f.removeNaNNormalsFromPointCloud(input:cdata(), output:cdata(), indices:cdata())
  return output
end

function filter.passThrough(input, fieldName, min, max, negative)
  local f, output = check_input_type(input)
  utils.check_arg('fieldName', type(fieldName) == 'string', 'string expected')
  min = min or pcl.range.double.min
  max = max or pcl.range.double.max
  f.passThrough(input:cdata(), output:cdata(), fieldName, min, max, negative or false)
  return output
end

function filter.cropBox(input, min, max, rotation, translation, transform, negative)
  local f, output = check_input_type(input)

  min = tensor(min)
  max = tensor(max)
  rotation = tensor(rotation)
  translation = tensor(translation)
  
  if torch.isTypeOf(transform, pcl.affine.Transform) then
    transform = transform:totensor()
  end
  
  transform = tensor(transform)

  f.cropBox(input:cdata(), output:cdata(), cdata(min), cdata(max), cdata(rotation), cdata(translation), cdata(transform), negative or false)

  return output
end

function filter.cropSphere(input, center, radius, transform, negative)
  local f, output = check_input_type(input)
  
  center = tensor(center)
  transform = tensor(transform)
  
  f.cropSphere(input:cdata(), output:cdata(), cdata(center), radius or 1, cdata(transform), negative or false)
  
  return output
end

function filter.voxelGrid(input, lx, ly, lz)
  local f, output = check_input_type(input)
  lx = lx or 1
  ly = ly or lx
  lz = lz or lx
  f.voxelGrid(input:cdata(), output:cdata(), lx, ly, lz)
  return output
end

function filter.statisticalOutlierRemoval(input, meanK, stddevMulThresh, negative)
  local f, output = check_input_type(input)
  f.statisticalOutlierRemoval(input:cdata(), output:cdata(), meanK or 2, stddevMulThresh or 1, negative or false)
  return output
end

function filter.randomSample(input, count)
  local f, output = check_input_type(input)
  f.randomSample(input:cdata(), output:cdata(), count or pcl.range.uint32.max)
  return output
end

function filter.medianFilter(input, windowSize)
  local f, output = check_input_type(input)
  f.medianFilter(input:cdata(), output:cdata(), windowSize or 5)
  return output
end

function filter.radiusOutlierRemoval(input, radius, minNeighbors, negative)
  local f, output = check_input_type(input)
  f.radiusOutlierRemoval(input:cdata(), output:cdata(), radius or 1, minNeighbors or 1, negative or false)
  return output
end

function filter.voxelHistogram(input, w, h, t, voxelSize, origin, center)
  local f = check_input_type(input, true)
  local output = torch.FloatTensor()
  origin = origin or {0,0,0}
  local count = f.voxelHistogram(input:cdata(), output:cdata(), w, h, t, voxelSize or 1, origin[1], origin[2], origin[3], center or false)
  return output, count
end
