local torch = require 'torch'
local pcl = require 'pcl.PointTypes'

local affine = {}

affine.UnitX = torch.FloatTensor({1,0,0,1})
affine.UnitY = torch.FloatTensor({0,1,0,1})
affine.UnitZ = torch.FloatTensor({0,0,1,1})

function affine.identity()
  return torch.eye(4):float()
end

function affine.invert(M)
  return torch.invert(M)
end

function affine.scale(x, y, z)
  if type(x) == 'table' then
    x,y,z = x[1],x[2],x[3]
  end  
  local S = torch.FloatTensor(4, 4):zero()
  S[{1,1}] = x
  S[{2,2}] = y
  S[{3,3}] = z
  S[{4,4}] = 1
  return S
end

function affine.translate(x, y, z)
  if type(x) == 'table' then
    x,y,z = x[1],x[2],x[3]
  end  
  local T = torch.FloatTensor(4, 4):zero()
  T[{1,4}] = x
  T[{2,4}] = y
  T[{3,4}] = z
  T[{4,4}] = 1
  return T
end

function affine.rotateEuler(x, y, z)
  if type(x) == 'table' then
    x,y,z = x[1],x[2],x[3]
  end
  local cos,sin = math.cos,math.sin
  local X = torch.FloatTensor({
    {       1,       0,       0,        0 },
    {       0,  cos(x), -sin(x),        0 },
    {       0,  sin(x),  cos(x),        0 },
    {       0,       0,       0,        1 }
  })
  local Y = torch.FloatTensor({
    {  cos(y),       0,  sin(y),       0 },
    {       0,       1,       0,       0 },
    { -sin(y),       0,  cos(y),       0 },
    {       0,       0,       0,       1 }
  })
  local Z = torch.FloatTensor({
    {  cos(z), -sin(z),       0,       0 },
    {  sin(z),  cos(z),       0,       0 },
    {       0,       0,       1,       0 },
    {       0,       0,       0,       1 }
  })
  return X * Y * Z
end

function affine.rotateAxis(axis, theta)
  if not torch.isTensor(axis) then
    axis = torch.FloatTensor(axis)
  end
  local n = axis:norm()
  if n == 0 then
    error('axis must not be the null vector')
  end
  local u = torch.div(axis, n)
  local ct, st = math.cos(theta), math.sin(theta)
  local d = 1-ct
  local R = torch.FloatTensor({
    {      ct+u[1]*u[1]*d, u[1]*u[2]*d-u[3]*st, u[1]*u[3]*d+u[2]*st, 0 },
    { u[2]*u[1]*d+u[3]*st,      ct+u[2]*u[2]*d, u[2]*u[3]*d-u[1]*st, 0 },
    { u[3]*u[1]*d-u[2]*st, u[3]*u[2]*d+u[1]*st,      ct+u[3]*u[3]*d, 0 },
    { 0, 0, 0, 1 },
  })
  return R
end

--[[
mode: e = euler, a = axis, q = quaternion
'e', x, y, z
'e', v
'a' x, y, z, theta
'a' v, theta
]]
function affine.rotate(mode, ...)
  if mode == 'e' or mode == 'ed' then
    local x,y,z
    if select('#', ...) == 1 then
      local v = select(1, ...)  -- table or tensor
      x,y,z = v[1],v[2],v[3]
    else
      x,y,z = unpack(...)
    end
    if mode == 'ed' then
      x,y,z = math.rad(x),math.rad(y),math.rad(z)
    end
    return affine.rotateEuler(x,y,z)
  elseif mode == 'a' or mode == 'ad' then
    local v, theta
    if select('#', ...) == 2 then
      v = select(1, ...)
      theta = select(2, ...)
    else
      local x = select(1, ...)
      local y = select(2, ...)
      local z = select(3, ...)
      v = {x,y,z}
      theta = select(4, ...)
    end
    if mode == 'ad' then
      theta = math.rad(theta)
    end
    return affine.rotateAxis(v, theta)
  elseif mode == 'q' then
    -- todo
  end
end

local Transform = torch.class('pcl.affine.Transform', affine)

function Transform:__init(M)
  self.M = M or affine.identity()
end

function Transform:premul(X)
  if torch.isTypeOf(X, affine.Transform) then
    X = X:totensor()
  end
  self.M:addmm(X, self.M)
end

function Transform:postmul(X)
  if torch.isTypeOf(X, affine.Transform) then
    X = X:totensor()
  end
  self.M:addmm(self.M, X)
end

function Transform:totensor()
  return self.M
end

function Transform:__tostring()
  local s = ''
  for i=1,4 do
    s = s .. string.format('%9g %9g %9g %9g\n', self.M[{i,1}], self.M[{i,2}], self.M[{i,3}], self.M[{i,4}])
  end
  return s
end

local TRANSFORM_NAMES = { 
  'scale', 
  'translate',
  'rotate'
}

for i,n in ipairs(TRANSFORM_NAMES) do
  local f = affine[n]
  Transform[n] = function(self, ...)
    self:postmul(f(...))
    return self
  end
  Transform['pre' .. n] = function(self, ...)
    self:premul(f(...))
    return self
  end
end

pcl.affine = affine
