local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local Indices = torch.class('pcl.Indices', pcl)

local methods = {}

local function init()
  local Indices_method_names = {
    'new',
    'clone',
    'delete',
    'size',
    'capacity',
    'reserve',
    'viewAsTensor',
    'copyToTensor',
    'copyFromTensor',
    'insertFromTensor',
    'getat',
    'setat',
    'push_back',
    'pop_back',
    'clear',
    'insert',
    'erase'
  }

  -- use utility function to automatically register new/clone results for GC delete
  methods = utils.create_typed_methods("pcl_PointIndices_", Indices_method_names, '')
end

init()

function Indices:__init(x)
  rawset(self, 'f', methods)
  rawset(self, 'o', self.f.new())
  
  if type(x) == 'table' then
    x = torch.IntTensor(x)
  end
  if torch.isTensor(x) then
    self:insertFromTensor(x)
  end
end

function Indices:cdata()
  return self.o
end

function Indices:clone()
  return self.f.clone(self.o)
end

function Indices:size()
  return self.f.size(self.o)
end

function Indices:__len()
  return self:size()
end

function Indices:capacity()
  return self.f.capacity(self.o)
end

function Indices:reserve(capacity)
  self.f.reserve(self.o, capacity)
end

function Indices:viewAsTensor(t)
  t = t or torch.IntTensor()
  self.f.viewAsTensor(self.o, t:cdata())
  return t
end

function Indices:copyToTensor(dst_tensor, src_index, dst_index, count)
  self.f.copyToTensor(self.o, dst_tensor:cdata(), (src_index or 1)-1, (dst_index or 1)-1, count or self:size())
end

function Indices:copyFromTensor(src_tensor, src_index, dst_index, count)
  self.f.copyFromTensor(self.o, src_tensor:cdata(), (src_index or 1)-1, (dst_index or 1)-1, count or src_tensor:size(1))
end

function Indices:insertFromTensor(src_tensor, src_index, dst_index, count)
  self.f.insertFromTensor(self.o, src_tensor:cdata(), (src_index or 1)-1, (dst_index or 1)-1, count or src_tensor:size(1))
end

function Indices:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = Indices[idx]
    if not v and type(idx) == 'number' then
      local f, o = rawget(self, 'f'), rawget(self, 'o')
      v = f.getat(o, idx-1)
    end
  end
  return v
end

function Indices:__newindex(idx, v)
  local f, o = rawget(self, 'f'), rawget(self, 'o')
  if type(idx) == 'number' then
    f.setat(o, idx-1, v)
  else
    rawset(self, idx, v)
  end
end

function Indices:push_back(value)
  self.f.push_back(self.o, value)
end

function Indices:pop_back()
  self.f.pop_back(self.o)
end

function Indices:clear()
  self.f.clear(self.o)
end

function Indices:insert(pos, value, n)
  self.f.insert(self.o, pos-1, n or 1, value)
end

function Indices:erase(begin_pos, end_pos)
  self.f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end
