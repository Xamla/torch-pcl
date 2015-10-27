local PointCloud = torch.class('pcl.PointCloud')

local pcl
local ffi
local func_by_type = {}

function init(_pcl, _ffi, p, type_key_map)
  pcl = _pcl
  ffi = _ffi
  
  local PointCloud_method_names = {
    "new",
    "clone",
    "delete",
    "width",
    "height",
    "isDense",
    "at1D",
    "at2D",
    "clear",
    "empty",
    "isOrganized",
    "points",
    "sensorOrientation",
    "sensorOrigin", 
    "add",
    "fromPCLPointCloud2",
    "loadPCDFile",
    "savePCDFile",
    "loadPLYFile",
    "savePLYFile",
    "loadOBJFile",
    "savePNGFile"
  }

  local generic_names = {}
  for i,n in ipairs(PointCloud_method_names) do
    generic_names[n] = "pcl_PointCloud_TYPE_KEY_" .. n
  end

  local function create_typed_methods(type_key)
    local map = {}
    for k,v in pairs(generic_names) do
      map[k] = p[string.gsub(v, "TYPE_KEY", type_key)]
    end
    return map
  end

  for k,v in pairs(type_key_map) do
    func_by_type[k] = create_typed_methods(v)
  end
end

function PointCloud:__init(pointType, width, height)
  pointType = pointType or pcl.PointXYZ
  width = width or 0
  height = height or 0
  self.pointType = pointType
  self.f = func_by_type[pointType]
  if torch.isTensor(width) then
    local sz = width:size()
    local w,h
    if width:nDimension() == 3 then
      w, h = sz[2], sz[1]
    elseif width:nDimension() == 2 then
      w, h = sz[1], 1
    end
    self.c = self.f.new(w, h)
    ffi.gc(self.c, self.f.delete)
    self:points():copy(width)
  elseif type(width) == 'cdata' then
    self.c = width
  else
    self.c = self.f.new(width, height)
    ffi.gc(self.c, self.f.delete)
  end
end

function PointCloud:clone()
  local clone = self.f.clone(self.c)
  ffi.gc(clone, self.f.delete)
  return PointCloud.new(self.pointType, clone)
end

function PointCloud:__index(column, row)
  local v = PointCloud[column]
  if not v and type(column) == 'number' then
    local f = self.f
    local c = self.c
    if not row then
      v = f.at1D(c, column-1)
    else
      v = f.at2D(c, column-1, row-1)
    end
  end
  return v
end

function PointCloud:width()
  return self.f.width(self.c)
end

function PointCloud:height()
  return self.f.height(self.c)
end

function PointCloud:isDense()
  return self.f.isDense(self.c)
end

function PointCloud:clear()
  self.f.clear(self.c)
end

function PointCloud:empty()
  return self.f.empty(self.c)
end

function PointCloud:isOrganized()
  return self.f.isOrganized(self.c)
end

function PointCloud:points()
  local t = torch.FloatTensor()
  local buf = self.f.points(self.c)
  t:cdata().storage = buf.storage
  t:resize(buf.height, buf.width, buf.dim)
  return t
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

function PointCloud:add(other)
  self.f.add(self.c, other.c)
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
  return self.f.savePCDFile(self.c, fn, binary)
end

function PointCloud:loadPLYFile(fn)
  return self.f.loadPLYFile(self.c, fn)
end

function PointCloud:savePLYFile(fn, binary)
  return self.f.savePLYFile(self.c, fn, binary)
end

function PointCloud:loadOBJFile(fn)
  return self.f.loadOBJFile(self.c, fn)
end
  
function PointCloud:savePNGFile(fn, field_name)
  return self.f.savePNGFile(self.c, fn)
end

function PointCloud:__tostring()
  return string.format("PointCloud (w:%d, h:%d)", self:width(), self:height())
end

return init