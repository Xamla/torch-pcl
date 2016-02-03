local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local Correspondences = torch.class('pcl.Correspondences', pcl)

local methods = {}

local function init()
  local Correspondences_method_names = {
    'new',
    'clone',
    'delete',
    'size',
    'at',
    'push_back',
    'pop_back',
    'clear',
    'insert',
    'erase',
    'empty'
  }
  
  -- use utility function to automatically register new/clone results for GC delete
  methods = utils.create_typed_methods("pcl_Correspondences_", Correspondences_method_names, '')
end

init()

function Correspondences:__init()
  rawset(self, 'f', methods)
  rawset(self, 'o', self.f.new())
end

function Correspondences:cdata()
  return self.o
end

function Correspondences:clone()
  return self.f.clone(self.o)
end

function Correspondences:size()
  return self.f.size(self.o)
end

function Correspondences:__len()
  return self:size()
end

function Correspondences:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = Correspondences[idx]
    if not v and type(idx) == 'number' then
      local f, o = rawget(self, 'f'), rawget(self, 'o')
      v = f.at(o, idx-1)
    end
  end
  return v
end

function Correspondences:__newindex(idx, v)
  local f, o = rawget(self, 'f'), rawget(self, 'o')
  if type(idx) == 'number' then
    f.at(o, idx-1):set(v)
  else
    rawset(self, idx, v)
  end
end

function Correspondences:push_back(value)
  self.f.push_back(self.o, value)
end

function Correspondences:pop_back()
  return self.f.pop_back(self.o)
end

function Correspondences:clear()
  self.f.clear(self.o)
end

function Correspondences:insert(pos, value)
  self.f.insert(self.o, pos-1, value)
end

function Correspondences:erase(begin_pos, end_pos)
  self.f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function Correspondences:empty()
  return self.f.empty(self.o)
end

function Correspondences:__pairs()
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

function Correspondences:__ipairs()
  return self:__pairs()
end

function Correspondences:__tostring()
  local t = {}
  for i,v in ipairs(self) do
    table.insert(t, tostring(v))
  end
  table.insert(t, string.format("[pcl.Correspondences of size %d]", #self))
  return table.concat(t, '\n')
end
