local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local filter = {}
pcl.filter = filter

local func_by_type = {}

local function init()

  local method_names = {
    'extractIndices',
    'shadowPoints_Indices',
    'shadowPoints_Cloud',
    'removeNaNFromPointCloud',
    'removeNaNNormalsFromPointCloud',
    'normalSpaceSampling_Indices',
    'normalSpaceSampling_Cloud',
    'normalRefinement',
    'frustumCulling_Indices',
    'frustumCulling_Cloud',
    'passThrough_Indices',
    'passThrough_Cloud',
    'cropBox_Indices',
    'cropBox_Cloud',
    'cropSphere_Indices',
    'cropSphere_Cloud',
    'voxelGrid',
    'statisticalOutlierRemoval_Indices',
    'statisticalOutlierRemoval_Cloud',
    'randomSample_Indices',
    'randomSample_Cloud',
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

local function check_input_type(input)
  utils.check_arg('input', pcl.isPointCloud(input), 'point cloud expected')
  return func_by_type[input.pointType]
end

local function tensor(x)
  if torch.isTensor(x) then
    x = x:float()  -- ensure we are dealing with a float tensor
  elseif pcl.isPoint(x) then
    x = x:totensor()
  elseif type(x) == 'table' then
    x = torch.FloatTensor(x)
  end
  return x
end

local function tensor4(x)
  x = tensor(x)
  if x:size(1) < 4 then
    local y = torch.FloatTensor(4)
    y:zero()
    y[4] = 1
    y[{{1,x:size(1)}}]:copy(x)
    x = y
  end
  return x
end

function filter.removeNaNFromPointCloud(input, output, removed_indices)
  local f = check_input_type(input)
  output = output or pcl.PointCloud(input.pointType)
  removed_indices = removed_indices or pcl.Indices()
  f.removeNaNFromPointCloud(input:cdata(), cdata(output), removed_indices:cdata())
  return output, removed_indices
end

function filter.removeNaNNormalsFromPointCloud(input, output, removed_indices)
  local f = check_input_type(input)
  utils.check_arg('input', f.removeNaNNormalsFromPointCloud, 'unsupported point type')
  removed_indices = removed_indices or pcl.Indices()
  f.removeNaNNormalsFromPointCloud(input:cdata(), output:cdata(), removed_indices:cdata())
  return output, removed_indices
end

function filter.extractIndices(input, indices, output, negative, removed_indices)
  local f = check_input_type(input)
  output = output or pcl.PointCloud(input.pointType)
  f.extractIndices(input:cdata(), indices:cdata(), output:cdata(), negative or false, cdata(removed_indices))
  return output
end

function filter.shadowPoints(input, normals, threshold, indices, output, negative, removed_indices)
  local f = check_input_type(input)
  if torch.isTypeOf(output, pcl.Indices) then
    f.shadowPoints_Indices(input:cdata(), utils.cdata(indices), normals:cdata(), output:cdata(), threshold or 0.1, negative or false, cdata(removed_indices))
  else
    output = output or pcl.PointCloud(input.pointType)
    f.shadowPoints_Cloud(input:cdata(), utils.cdata(indices), normals:cdata(), output:cdata(), threshold or 0.1, negative or false, cdata(removed_indices))
  end
  return output
end

function filter.normalSpaceSampling(input, normals, samples, binsx, binsy, binsz, indices, output)
  local f = check_input_type(input)
  if torch.isTypeOf(output, pcl.Indices) then
    f.normalSpaceSampling_Indices(input:cdata(), cdata(indices), normals:cdata(), output:cdata(), samples or 10000, binsx or 50, binsy or 50, binsz or 50)
  else
    output = output or pcl.PointCloud(input.pointType)
    f.normalSpaceSampling_Cloud(input:cdata(), cdata(indices), normals:cdata(), output:cdata(), samples or 10000, binsx or 50, binsy or 50, binsz or 50)
  end
  return output
end

function filter.normalRefinement(input, k, max_iterations, convergence_threshold, output)
  local f = check_input_type(input)
  output = output or pcl.PointCloud(input.pointType)
  f.normalRefinement(input:cdata(), output, k or 5, max_iterations or 15, convergence_threshold or 0.00001)
  return output
end

function filter.frustumCulling(input, cameraPose, hfov, vfov, np_dist, fp_dist, indices, output, negative, removed_indices)
  local f = check_input_type(input)
  if torch.isTypeOf(output, pcl.Indices) then
    f.frustumCulling_Indices(input:cdata(), cdata(indices), output:cdata(), cameraPose:cdata(), hfov, vfov, np_dist, fp_dist, negative or false, cdata(removed_indices))
  else
    output = output or pcl.PointCloud(input.pointType)
    f.frustumCulling_Cloud(input:cdata(), cdata(indices), output:cdata(), cameraPose:cdata(), hfov, vfov, np_dist, fp_dist, negative or false, cdata(removed_indices))
  end
  return output
end

function filter.passThrough(input, fieldName, min, max, indices, output, negative, removed_indices, keepOrganized)
  local f = check_input_type(input)
  utils.check_arg('fieldName', type(fieldName) == 'string', 'string expected')
  min = min or pcl.range.double.min
  max = max or pcl.range.double.max

  if torch.isTypeOf(output, pcl.Indices) then
    f.passThrough_Indices(input:cdata(), cdata(indices), output:cdata(), fieldName, min, max, negative or false, cdata(removed_indices), keepOrganized or false)
  else
    output = output or pcl.PointCloud(input.pointType)
    f.passThrough_Cloud(input:cdata(), cdata(indices), output:cdata(), fieldName, min, max, negative or false, cdata(removed_indices), keepOrganized or false)
  end
  return output
end

function filter.cropBox(input, min, max, rotation, translation, transform, indices, output, negative, removed_indices)
  local f = check_input_type(input)

  min = tensor4(min)
  max = tensor4(max)
  rotation = tensor(rotation)
  translation = tensor(translation)

  if torch.isTypeOf(transform, pcl.affine.Transform) then
    transform = transform:totensor()
  else
    transform = tensor(transform)
  end

  if torch.isTypeOf(output, pcl.Indices) then
    f.cropBox_Indices(input:cdata(), cdata(indices), output:cdata(), cdata(min), cdata(max), cdata(rotation), cdata(translation), cdata(transform), negative or false, cdata(removed_indices))
  else
    output = output or pcl.PointCloud(input.pointType)
    f.cropBox_Cloud(input:cdata(), cdata(indices), output:cdata(), cdata(min), cdata(max), cdata(rotation), cdata(translation), cdata(transform), negative or false, cdata(removed_indices))
  end

  return output
end

function filter.cropSphere(input, center, radius, transform, indices, output, negative, removed_indices)
  local f = check_input_type(input)

  center = tensor4(center)

  if torch.isTypeOf(transform, pcl.affine.Transform) then
    transform = transform:totensor()
  else
    transform = tensor(transform)
  end

  if torch.isTypeOf(output, pcl.Indices) then
    f.cropSphere_Indices(input:cdata(), cdata(indices), output:cdata(), cdata(center), radius or 1, cdata(transform), negative or false, cdata(removed_indices))
  else
    output = output or pcl.PointCloud(input.pointType)
    f.cropSphere_Cloud(input:cdata(), cdata(indices), output:cdata(), cdata(center), radius or 1, cdata(transform), negative or false, cdata(removed_indices))
  end
  return output
end

function filter.voxelGrid(input, lx, ly, lz, indices, output)
  local f = check_input_type(input)
  lx = lx or 1
  ly = ly or lx
  lz = lz or lx
  output = output or pcl.PointCloud(input.pointType)
  f.voxelGrid(input:cdata(), cdata(indices), output:cdata(), lx, ly, lz)
  return output
end

function filter.statisticalOutlierRemoval(input, meanK, stddevMulThresh, indices, output, negative, removed_indices)
  local f = check_input_type(input)
  if torch.isTypeOf(output, pcl.Indices) then
    f.statisticalOutlierRemoval_Indices(input:cdata(), cdata(indices), output:cdata(), meanK or 2, stddevMulThresh or 1, negative or false, cdata(removed_indices))
  else
    output = output or pcl.PointCloud(input.pointType)
    f.statisticalOutlierRemoval_Cloud(input:cdata(), cdata(indices), output:cdata(), meanK or 2, stddevMulThresh or 1, negative or false, cdata(removed_indices))
  end
  return output
end

function filter.randomSample(input, count, indices, output)
  local f = check_input_type(input)
  if torch.isTypeOf(output, pcl.Indices) then
    f.randomSample_Indices(input:cdata(), cdata(indices), output:cdata(), count or pcl.range.uint32.max)
  else
    output = output or pcl.PointCloud(input.pointType)
    f.randomSample_Cloud(input:cdata(), cdata(indices), output:cdata(), count or pcl.range.uint32.max)
  end
  return output
end

function filter.medianFilter(input, windowSize, indices, output)
  local f = check_input_type(input)
  output = output or pcl.PointCloud(input.pointType)
  f.medianFilter(input:cdata(), cdata(indices), output:cdata(), windowSize or 5)
  return output
end

function filter.radiusOutlierRemoval(input, radius, minNeighbors, indices, output, negative, removed_indices)
  local f = check_input_type(input)
  output = output or pcl.PointCloud(input.pointType)
  f.radiusOutlierRemoval(input:cdata(), cdata(indices), output:cdata(), radius or 1, minNeighbors or 1, negative or false, cdata(removed_indices))
  return output
end

function filter.voxelHistogram(input, w, h, t, voxelSize, origin, center, output)
  local f = check_input_type(input)
  if output and not torch.isTypeOf(output, torch.FloatTensor) then
    error("Invalid type of argument 'output': torch.FloatTensor expected.")
  end
  output = output or torch.FloatTensor()
  origin = origin or {0,0,0}
  local count = f.voxelHistogram(input:cdata(), output:cdata(), w, h, t, voxelSize or 1, origin[1], origin[2], origin[3], center or false)
  return output, count
end
