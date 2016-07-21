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
  return torch.inverse(M)
end

function affine.scale(x, y, z)
  if type(x) == 'table' or torch.isTensor(x) then
    x,y,z = x[1],x[2],x[3]
  end
  y = y or x
  z = z or x
  local S = torch.eye(4):float()
  S[{1,1}] = x
  S[{2,2}] = y
  S[{3,3}] = z
  return S
end

function affine.translate(x, y, z)
  if type(x) == 'table' or torch.isTensor(x) then
    x,y,z = x[1],x[2],x[3]
  end  
  local T = torch.eye(4):float()
  T[{1,4}] = x
  T[{2,4}] = y
  T[{3,4}] = z
  return T
end

function affine.rotateEuler(x, y, z, deg)
  if type(x) == 'table' or torch.isTensor(x) then
    x,y,z = x[1],x[2],x[3]
    deg = y
  end
  if deg then
    x = math.rad(x)
    y = math.rad(y)
    z = math.rad(z)
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

function affine.rotateAxis(axis, theta, deg)
  if not torch.isTensor(axis) then
    axis = torch.FloatTensor(axis)
  end
  if deg then
    theta = math.rad(theta)
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

local Transform = torch.class('pcl.affine.Transform', affine)

function Transform:__init(M)
  self.M = M or affine.identity()
end

function Transform:premul(X)
  if torch.isTypeOf(X, affine.Transform) then
    X = X:totensor()
  end
  self.M:set(X * self.M)
end

function Transform:postmul(X)
  if torch.isTypeOf(X, affine.Transform) then
    X = X:totensor()
  end
  self.M:set(self.M * X)
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
  'rotateEuler',
  'rotateAxis'
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
