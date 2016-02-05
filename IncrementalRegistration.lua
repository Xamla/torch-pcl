local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local IncrementalRegistration = torch.class('pcl.IncrementalRegistration', pcl)

local func_by_type = {}

local function init()

  local IncrementalRegistration_method_names = {
    'new',
    'delete',
    'setICP',
    'setICPNL',
    'registerCloud',
    'getDeltaTransform',
    'getAbsoluteTransform'
  }

  for k,v in pairs(utils.type_key_map) do
    func_by_type[k] = utils.create_typed_methods("pcl_IncrementalRegistration_TYPE_KEY_", IncrementalRegistration_method_names, v)
  end
end

init()

function IncrementalRegistration:__init(pointType)
  pointType = pointType or pcl.PointXYZ
  self.pointType = pcl.pointType(pointType)
  self.f = func_by_type[self.pointType]
  self.o = self.f.new()
end

function IncrementalRegistration:cdata()
  return self.o
end

function IncrementalRegistration:setRegistration(reg)
  if torch.isTypeOf(reg, pcl.ICP) then
    self.f.setICP(self.o, reg:cdata())
  elseif torch.isTypeOf(reg, pcl.ICPNL) then
    self.f.setICPNL(self.o, reg:cdata())
  else
    utils.check_arg('reg', false, 'Unsupported registration algorithm obj')
  end
end

function IncrementalRegistration:reset()
  self.f.reset(self.o)
end

function IncrementalRegistration:registerCloud(cloud, delta_estimate)
  return self.f.registerCloud(self.o, cloud:cdata(), delta_estimate)
end

function IncrementalRegistration:getDeltaTransform()
  local t = torch.FloatTensor()
  self.f.getDeltaTransform(self.o, t:cdata())
  return t
end

function IncrementalRegistration:getAbsoluteTransform()
  local t = torch.FloatTensor()
  self.f.getAbsoluteTransform(self.o, t:cdata())
  return t
end
