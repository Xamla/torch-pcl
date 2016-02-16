local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local PointCloud = torch.class('pcl.PointCloud', pcl)

local func_by_type = {}

local function init()
  local PointCloud_method_names = {
    'new',
    'clone',
    'delete',
    'getWidth',
    'getHeight',
    'getIsDense',
    'setIsDense',
    'at1D',
    'at2D',
    'clear',
    'reserve',
    'size',
    'empty',
    'isOrganized',
    'push_back',
    'insert',
    'erase',
    'points',
    'sensorOrientation',
    'sensorOrigin', 
    'transform',
    'transformWithNormals',
    'getMinMax3D',
    'compute3DCentroid',
    'computeCovarianceMatrix',
    'add',
    'fromPCLPointCloud2',
    'loadPCDFile',
    'savePCDFile',
    'loadPLYFile',
    'savePLYFile',
    'loadOBJFile',
    'savePNGFile',
    'readXYZfloat',
    'readRGBAfloat',
    'readRGBAbyte',
    'addNormals',
    'copyXYZ',
    'copyXYZI',
    'copyXYZRGBA',
    'copyXYZNormal',
    'copyXYZINormal',
    'copyXYZRGBNormal',
    'copyNormal'
  }

  local supported_types = {}
  supported_types[pcl.PointXYZ] = 'XYZ'
  supported_types[pcl.PointXYZI] = 'XYZI'
  supported_types[pcl.PointXYZRGBA] = 'XYZRGBA'
  supported_types[pcl.PointNormal] = 'XYZNormal'
  supported_types[pcl.PointXYZINormal] = 'XYZINormal'
  supported_types[pcl.PointXYZRGBNormal] = 'XYZRGBNormal'
  supported_types[pcl.Normal] = 'Normal'
  supported_types[pcl.FPFHSignature33] = 'FPFHSignature33'
  supported_types[pcl.VFHSignature308] = 'VFHSignature308'
  supported_types[pcl.Boundary] = 'Boundary'
  supported_types[pcl.Label] = 'Label'
  for k,v in pairs(supported_types) do
    func_by_type[k] = utils.create_typed_methods('pcl_PointCloud_TYPE_KEY_', PointCloud_method_names, v)
  end
  
  func_by_type[pcl.Normal] = utils.create_typed_methods('pcl_PointCloud_TYPE_KEY_', PointCloud_method_names, 'Normal')
end

init()

function PointCloud:__init(pointType, width, height)
  if type(pointType) == 'number' then
    width = pointType
    pointType = pcl.PointXYZ
  end
  
  pointType = pcl.pointType(pointType)
  width = width or 0
  rawset(self, 'f', func_by_type[pointType])
  self.pointType = pointType
  if torch.isTensor(width) then
    width = width:float()
    local sz = width:size()
    local w,h
    if width:nDimension() == 3 then
      w, h = sz[2], sz[1]
    elseif width:nDimension() == 2 then
      w, h = sz[1], 1
    end
    self.o = self.f.new(w, h)
    self:points():copy(width)
  elseif type(width) == 'cdata' then
    self.o = width
  else
    height = height or width and width > 0 and 1 or 0
    self.o = self.f.new(width, height)
  end
end

function PointCloud:cdata()
  return self.o
end

function PointCloud:readXYZ(t)
  local t = t or torch.FloatTensor()
  if torch.type(t) == 'torch.FloatTensor' then
    self.f.readXYZfloat(self.o, t:cdata())
  else
    error('torch.FloatTensor expected')
  end
  return t
end

function PointCloud:readRGBA(t)
  local t = t or torch.FloatTensor()
  if torch.type(t) == 'torch.FloatTensor' then
    self.f.readRGBAfloat(self.o, t:cdata())
  elseif torch.type(t) == 'torch.ByteTensor' then
    self.f.readRGBAbyte(self.o, t:cdata())
  else
    error('unsupported tensor type')
  end
  return t
end

function PointCloud:clone()
  local clone = self.f.clone(self.o)
  return PointCloud.new(self.pointType, clone)
end

function PointCloud:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = PointCloud[idx]
    if not v then
      local f, o = rawget(self, 'f'), rawget(self, 'o')
      if type(idx) == 'number' then
        v = f.at1D(o, idx-1)
      elseif type(idx) == 'table' then
        v = f.at2D(o, idx[1]-1, row[2]-1)
      end
    end
  end
  return v
end

function PointCloud:__newindex(idx, v)
  local f, o = rawget(self, 'f'), rawget(self, 'o')
  if type(idx) == 'number' then
    f.at1D(o, idx-1):set(v)
  elseif type(idx) == 'table' then
    f.at2D(o, idx[1]-1, row[2]-1):set(v)
  else
    rawset(self, idx, v)
  end
end

function PointCloud:__len()
  return self:size()
end

function PointCloud:getWidth()
  return self.f.getWidth(self.o)
end

function PointCloud:getHeight()
  return self.f.getHeight(self.o)
end

function PointCloud:getIsDense()
  return self.f.getIsDense(self.o)
end

function PointCloud:setIsDense(value)
  self.f.setIsDense(self.o, value)
end

function PointCloud:clear()
  self.f.clear(self.o)
end

function PointCloud:reserve(n)
  self.f.reserve(self.o, n)
end

function PointCloud:size()
  return self.f.size(self.o);
end

function PointCloud:empty()
  return self.f.empty(self.o)
end

function PointCloud:isOrganized()
  return self.f.isOrganized(self.o)
end

function PointCloud:push_back(pt)
  self.f.push_back(self.o, pt);
end

function PointCloud:insert(pos, pt, n)
  self.f.insert(self.o, pos-1, n or 1, pt)
end

function PointCloud:erase(begin_pos, end_pos)
  self.f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function PointCloud:points()
  local t = torch.FloatTensor()
  local buf = self.f.points(self.o)
  t:cdata().storage = buf.storage
  t:resize(buf.height, buf.width, buf.dim)
  return t
end

function PointCloud:pointsXYZ()
  return self:points()[{{},{},{1,3}}]
end

function PointCloud:sensorOrigin()
  local t = torch.FloatTensor()
  local s = self.f.sensorOrigin(self.o)
  t:cdata().storage = s
  t:resize(4)
  return t
end

function PointCloud:sensorOrientation()
  local t = torch.FloatTensor()
  local s = self.f.sensorOrientation(self.o)
  t:cdata().storage = s
  t:resize(4)
  return t
end

function PointCloud:transform(mat, output)
  if torch.isTypeOf(mat, pcl.affine.Transform) then
    mat = mat:totensor()
  end
  output = output or self
  if self.pointType.hasNormal then
    self.f.transformWithNormals(self.o, mat:cdata(), output:cdata())
  else
    self.f.transform(self.o, mat:cdata(), output:cdata())
  end
  return output
end

function PointCloud:getMinMax3D()
  local min, max = self.pointType(), self.pointType()
  self.f.getMinMax3D(self.o, min, max)
  return min, max
end

function PointCloud:compute3DCentroid()
  local centroid = torch.FloatTensor()
  self.f.compute3DCentroid(self.o, centroid:cdata())
  return centroid
end

function PointCloud:computeCovarianceMatrix(centroid)
  local covariance = torch.FloatTensor()
  if not centroid then
    centroid = self:compute3DCentroid()
  end
  self.f.computeCovarianceMatrix(self.o, utils.cdata(centroid), covariance:cdata())
  return covariance, centroid
end

function PointCloud:add(other)
  self.f.add(self.o, other.o)
end

function PointCloud:removeNaN(output, removed_indices)
  if torch.isTypeOf(output, pcl.Indices) then
    return pcl.filter.removeNaNFromPointCloud(self, nil, output)
  else
    return pcl.filter.removeNaNFromPointCloud(self, output or self, removed_indices)
  end
end

function PointCloud:fromPCLPointCloud2(src_msg)
  self.f.fromPCLPointCloud2(self.o, src_msg)
end

function PointCloud:toPCLPointCloud2(dst_msg)
  self.f.toPCLPointCloud2(self.o, dst_msg)
end

function PointCloud:loadPCDFile(fn)
  return self.f.loadPCDFile(self.o, fn)
end

function PointCloud:savePCDFile(fn, binary)
  return self.f.savePCDFile(self.o, fn, binary or true)
end

function PointCloud:loadPLYFile(fn)
  return self.f.loadPLYFile(self.o, fn)
end

function PointCloud:savePLYFile(fn, binary)
  return self.f.savePLYFile(self.o, fn, binary or true)
end

function PointCloud:loadOBJFile(fn)
  return self.f.loadOBJFile(self.o, fn)
end
  
function PointCloud:savePNGFile(fn, field_name)
  return self.f.savePNGFile(self.o, fn, field_name or 'rgb')
end

function PointCloud:addNormals(normals, output)
  if not output then
    output = pcl.PointCloud(utils.getNormalTypeFor(self.pointType))
  end
  self.f.addNormals(self.o, normals:cdata(), output:cdata())
  return output
end

function PointCloud:__tostring()
  return string.format('PointCloud<%s> (w: %d, h: %d, organized: %s, dense: %s)', 
    pcl.getPointTypeName(self.pointType),
    self:getWidth(),
    self:getHeight(),
    self:isOrganized(),
    self:getIsDense()
  )
end

function PointCloud:apply(func)
  local p = self:points()
  local count = p:nElement()
  local data = p:data()
  local pt = self.pointType()
  local point_size = ffi.sizeof(pt)
  local stride =  point_size / #pt
  local j=1
  for i=0,count-1,stride do
    ffi.copy(pt, data + i, point_size)
    local r = func(pt, j)   -- pass point and index to function
    if r then
      ffi.copy(data + i, r, point_size)
    end
    j = j + 1
  end
end

function PointCloud.copy(cloud_in, indices, cloud_out)
  if torch.isTypeOf(indices, pcl.PointCloud) then
    cloud_out = indices
    indices = nil
  end
  if not cloud_out then
    cloud_out = pcl.PointCloud(cloud_in.pointType)
  end
  local copy = cloud_in.f["copy" .. (utils.type_key_map[cloud_out.pointType] or '')]
  if not copy then
    print("copy" .. (utils.type_key_map[cloud_out.pointType] or ''))
    error("Copy to destination point cloud type not supported.")
  end
  copy(cloud_in:cdata(), utils.cdata(indices), cloud_out:cdata())
  return cloud_out
end
