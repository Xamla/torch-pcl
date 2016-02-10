local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local Indices = torch.class('pcl.Indices', pcl)
local Indices_factory = torch.factory('pcl.Indices')

local methods = {}

local function init()
  local Indices_method_names = {
    'new',
    'clone',
    'delete',
    'fromPtr',
    'size',
    'capacity',
    'reserve',
    'append',
    'insertMany',
    'viewAsTensor',
    'copyToTensor',
    'copyFromTensor',
    'insertFromTensor',
    'getAt',
    'setAt',
    'push_back',
    'pop_back',
    'clear',
    'insert',
    'erase'
  }

  -- use utility function to automatically register new/clone results for GC delete
  methods = utils.create_typed_methods("pcl_Indices_", Indices_method_names, '')
end

init()

function Indices:__init(x)
  rawset(self, 'f', methods)
  rawset(self, 'o', self.f.new())
  
  if type(x) == 'table' then
    x = torch.IntTensor(x)-1
  end
  if torch.isTensor(x) then
    self:insertFromTensor(x)
  end
end

function Indices.fromPtr(x)
  local y = Indices_factory()
  rawset(y, 'f', methods)
  rawset(y, 'o', methods.fromPtr(x))
  ffi.gc(y.o, methods.delete)   -- register cleanup function
  return y
end

function Indices:cdata()
  return self.o
end

function Indices:clone()
  local c = Indices_factory()
  rawset(c, 'f', self.f)
  rawset(c, 'o', self.f.clone(self.o))
  return c
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

function Indices:append(source)
  self.f.append(self.o, source:cdata())
end

function Indices:insertMany(source, src_index, dst_index, count)
  self.f.insertMany(self.o, source:cdata(), (src_index or 1)-1, (dst_index or 1)-1, count or source:size())
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
      v = f.getAt(o, idx-1) + 1
    end
  end
  return v
end

function Indices:__newindex(idx, v)
  local f, o = rawget(self, 'f'), rawget(self, 'o')
  if type(idx) == 'number' then
    f.setAt(o, idx-1, v-1)
  else
    rawset(self, idx, v)
  end
end

function Indices:push_back(value)
  self.f.push_back(self.o, value-1)
end

function Indices:pop_back()
  return self.f.pop_back(self.o)
end

function Indices:clear()
  self.f.clear(self.o)
end

function Indices:insert(pos, value, n)
  if torch.isTypeOf(value, pcl.Indices) then
    self:insertMany(value:cdata(), 1, pos, n or value:size())
  else
    self.f.insert(self.o, pos-1, n or 1, value)
  end
end

function Indices:erase(begin_pos, end_pos)
  self.f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function Indices:__pairs()
  return function (t, k)
    local i = k or 1
    if i > #t then
      return nil
    else
      local v = t[i]
      return i+1, v
    end
  end, self, nil
end

function Indices:__ipairs()
  return self:__pairs()
end

function Indices:totable()
  local t = {}
  for i,v in ipairs(self) do
    table.insert(t, v)
  end
  return t
end

function Indices:__tostring()
  local t = self:totable()
  table.insert(t, string.format("[pcl.Indices of size %d]", #self))
  return table.concat(t, '\n')
end
