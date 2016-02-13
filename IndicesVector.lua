local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local IndicesVector = torch.class('pcl.IndicesVector', pcl)

local methods = {}

local function init()
  local IndicesVector_method_names = {
    'new',
    'delete',
    'size',
    'getAt',
    'setAt',
    'push_back',
    'pop_back',
    'clear',
    'insert',
    'erase',
    'empty'
  }

  -- use utility function to automatically register new/clone results for GC delete
  methods = utils.create_typed_methods("pcl_IndicesVector_", IndicesVector_method_names, '')
end

init()

function IndicesVector:__init()
  rawset(self, 'f', methods)
  rawset(self, 'o', self.f.new())
end

function IndicesVector:cdata()
  return self.o
end

function IndicesVector:size()
  return self.f.size(self.o)
end

function IndicesVector:__len()
  return self:size()
end

function IndicesVector:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = IndicesVector[idx]
    if not v and type(idx) == 'number' then
      local f, o = rawget(self, 'f'), rawget(self, 'o')
      v = pcl.Indices()
      f.getAt(o, idx-1, v:cdata())
    end
  end
  return v
end

function IndicesVector:__newindex(idx, v)
  local f, o = rawget(self, 'f'), rawget(self, 'o')
  if type(idx) == 'number' then
    f.setAt(o, idx-1, v:cdata())
  else
    rawset(self, idx, v)
  end
end

function IndicesVector:push_back(value)
  self.f.push_back(self.o, value:cdata())
end

function IndicesVector:pop_back()
  self.f.pop_back(self.o)
end

function IndicesVector:clear()
  self.f.clear(self.o)
end

function IndicesVector:insert(pos, value)
  self.f.insert(self.o, pos, value:cdata())
end

function IndicesVector:erase(begin_pos, end_pos)
  self.f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function IndicesVector:__tostring()
  return string.format("[pcl.IndicesVector of size %d]", #self)
end

