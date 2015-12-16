local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local PointCloud = torch.class('pcl.PointCloud', pcl)

local func_by_type = {}

local function init()
  local PointCloud_method_names = {
    "new",
    "clone",
    "delete",
    "getWidth",
    "getHeight",
    "getIsDense",
    "setIsDense",
    "at1D",
    "at2D",
    "clear",
    "reserve",
    "size",
    "empty",
    "isOrganized",
    "push_back",
    "insert",
    "erase",
    "points",
    "sensorOrientation",
    "sensorOrigin", 
    "transform",
    "getMinMax3D",
    "add",
    "fromPCLPointCloud2",
    "loadPCDFile",
    "savePCDFile",
    "loadPLYFile",
    "savePLYFile",
    "loadOBJFile",
    "savePNGFile",
    "readXYZfloat",
    "readRGBAfloat",
    "readRGBAbyte"
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_PointCloud_TYPE_KEY_", PointCloud_method_names, v)
  end
end

init()

function PointCloud:__init(pointType, width, height)
  if type(pointType) == 'number' then
    widthr = pointType
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
    self.c = self.f.new(w, h)
    self:points():copy(width)
  elseif type(width) == 'cdata' then
    self.c = width
  else
    height = height or width and width > 0 and 1 or 0
    self.c = self.f.new(width, height)
  end
end

function PointCloud:cdata()
  return self.c
end

function PointCloud:readXYZ(t)
  local t = t or torch.FloatTensor()
  if torch.type(t) == 'torch.FloatTensor' then
    self.f.readXYZfloat(self.c, t:cdata())
  else
    error('torch.FloatTensor expected')
  end
  return t
end

function PointCloud:readRGBA(t)
  local t = t or torch.FloatTensor()
  if torch.type(t) == 'torch.FloatTensor' then
    self.f.readRGBAfloat(self.c, t:cdata())
  elseif torch.type(t) == 'torch.ByteTensor' then
    self.f.readRGBAbyte(self.c, t:cdata())
  else
    error('unsupported tensor type')
  end
  return t
end

function PointCloud:clone()
  local clone = self.f.clone(self.c)
  return PointCloud.new(self.pointType, clone)
end

function PointCloud:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = PointCloud[idx]
    if not v then
      local f, c = rawget(self, 'f'), rawget(self, 'c')
      if type(idx) == 'number' then
        v = f.at1D(c, idx-1)
      elseif type(idx) == 'table' then
        v = f.at2D(c, idx[1]-1, row[2]-1)
      end
    end
  end
  return v
end

function PointCloud:__newindex(idx, v)
  local f, c = rawget(self, 'f'), rawget(self, 'c')
  if type(idx) == 'number' then
    f.at1D(c, idx-1):set(v)
  elseif type(idx) == 'table' then
    f.at2D(c, idx[1]-1, row[2]-1):set(v)
  else
    rawset(self, idx, v)
  end
end

function PointCloud:getWidth()
  return self.f.getWidth(self.c)
end

function PointCloud:getHeight()
  return self.f.getHeight(self.c)
end

function PointCloud:getIsDense()
  return self.f.getIsDense(self.c)
end

function PointCloud:setIsDense(value)
  self.f.setIsDense(self.c, value)
end

function PointCloud:clear()
  self.f.clear(self.c)
end

function PointCloud:reserve(n)
  self.f.reserve(self.c, n)
end

function PointCloud:size()
  return self.f.size(self.c);
end

function PointCloud:empty()
  return self.f.empty(self.c)
end

function PointCloud:isOrganized()
  return self.f.isOrganized(self.c)
end

function PointCloud:push_back(pt)
  self.f.push_back(self.c, pt);
end

function PointCloud:insert(pos, pt, n)
  self.f.insert(self.c, pos-1, pt, n or 1)
end

function PointCloud:erase(begin_pos, end_pos)
  self.f.erase(self.c, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function PointCloud:points()
  local t = torch.FloatTensor()
  local buf = self.f.points(self.c)
  t:cdata().storage = buf.storage
  t:resize(buf.height, buf.width, buf.dim)
  return t
end

function PointCloud:pointsXYZ()
  return self:points()[{{},{},{1,3}}]
end

function PointCloud:sensorOrigin()
  local t = torch.FloatTensor()
  local s = self.f.sensorOrigin(self.c)
  t:cdata().storage = s
  t:resize(4)
  return t
end

function PointCloud:sensorOrientation()
  local t = torch.FloatTensor()
  local s = self.f.sensorOrientation(self.c)
  t:cdata().storage = s
  t:resize(4)
  return t
end

function PointCloud:transform(mat, output)
  if torch.isTypeOf(mat, pcl.affine.Transform) then
    mat = mat:totensor()
  end
  output = output or self
  self.f.transform(self.c, mat:cdata(), output:cdata())
  return output
end

function PointCloud:getMinMax3D()
  local min, max = self.pointType(), self.pointType()
  self.f.getMinMax3D(self.c, min, max)
  return min, max
end

function PointCloud:add(other)
  self.f.add(self.c, other.c)
end

function PointCloud:removeNaN(indices, inplace)
  return pcl.filter.removeNaNFromPointCloud(self, indices, inplace or true)
end

function PointCloud:fromPCLPointCloud2(src_msg)
  self.f.fromPCLPointCloud2(self.c, src_msg)
end

function PointCloud:toPCLPointCloud2(dst_msg)
  self.f.toPCLPointCloud2(self.c, dst_msg)
end

function PointCloud:loadPCDFile(fn)
  return self.f.loadPCDFile(self.c, fn)
end

function PointCloud:savePCDFile(fn, binary)
  return self.f.savePCDFile(self.c, fn, binary or true)
end

function PointCloud:loadPLYFile(fn)
  return self.f.loadPLYFile(self.c, fn)
end

function PointCloud:savePLYFile(fn, binary)
  return self.f.savePLYFile(self.c, fn, binary or true)
end

function PointCloud:loadOBJFile(fn)
  return self.f.loadOBJFile(self.c, fn)
end
  
function PointCloud:savePNGFile(fn, field_name)
  return self.f.savePNGFile(self.c, fn)
end

function PointCloud:__tostring()
  return string.format("PointCloud (w:%d, h:%d)", self:getWidth(), self:getHeight())
end
