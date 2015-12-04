local torch = require 'torch'
local pcl = require 'pcl.PointTypes'

local Affine3 = {}

function Affine3.identity()
  return torch.eye(4)
end

function Affine3.translate(t, M)
  M = M or torch.eye(4)
  local T = torch.eye(4)
  T[{1,4}] = t[1]
  T[{2,4}] = t[2]
  T[{3,4}] = t[3]
  return T * M
end

function Affine3.scale(s, M)
  M = M or torch.eye(4)
  if type(s) == 'number' then
    s = { s, s, s }
  end
  local S = torch.eye(4)
  S[{1,1}] = s[1]
  S[{2,2}] = s[2]
  S[{3,3}] = s[3]
  return S * M
end

function Affine3.rotateEuler(x, y, z, M)
  M = M or torch.eye(4)
  local cos,sin = math.cos,math.sin
  local X = torch.Tensor({
    {       1,       0,       0,        0 },
    {       0,  cos(x), -sin(x),        0 },
    {       0,  sin(x),  cos(x),        0 },
    {       0,       0,       0,        1 }
  })
  local Y = torch.Tensor({
    {  cos(y),       0,  sin(y),       0 },
    {       0,       1,       0,       0 },
    { -sin(y),       0,  cos(y),       0 },
    {       0,       0,       0,       1 }
  })
  local Z = torch.Tensor({
    {  cos(z), -sin(z),       0,       0 },
    {  sin(z),  cos(z),       0,       0 },
    {       0,       0,       1,       0 },
    {       0,       0,       0,       1 }
  })
  local R = X * Y * Z
  return R * M
end

function Affine3.rotateAxis(axis, theta, M)
  M = M or torch.eye(4)
  if not torch.isTensor(axis) then
    axis = torch.Tensor(axis)
  end
  local n = axis:norm()
  if n == 0 then
    error('axis must not be the null vector')
  end
  local u = torch.div(axis, n)
  local ct, st = math.cos(theta), math.sin(theta)
  local R = torch.Tensor({
    {      ct+u[1]*u[1]*(1-ct), u[1]*u[2]*(1-ct)-u[3]*st, u[1]*u[3]*(1-ct)+u[2]*st, 0 },
    { u[2]*u[1]*(1-ct)+u[3]*st,      ct+u[2]*u[2]*(1-ct), u[2]*u[3]*(1-ct)-u[1]*st, 0 },
    { u[3]*u[1]*(1-ct)-u[2]*st, u[3]*u[2]*(1-ct)+u[1]*st,      ct+u[3]*u[3]*(1-ct), 0 },
    { 0, 0, 0, 1 },
  })
  return R * M
end

function Affine3.makeAffine(M)
  local M = M:clone()
  M:resize(4, 4)
  M[{4,{1,3}}] = 0
  M[{4,4}] = 1
  return M;
end

pcl.Affine3 = Affine3
