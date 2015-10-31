require 'torch'
local ffi = require 'ffi'

pcl = {}

require 'PointTypes'
require 'PointCloud'
require 'CloudViewer'
require 'OpenNI2Stream'

--[[
local obj = PointCloud.new(pcl.PointXYZ, torch.FloatTensor({{ 4,3,2,1 }, { 5,3,2, 1 }}))
print(obj:points())
print(obj[1])
print(obj[2])


local obj2 = PointCloud.new(pcl.PointXYZ, 3, 3)
print(obj2:points())
obj:add(obj2)
print(obj:points())
]]


-- TODO:
-- loading a point cloud file and get points as torch tensors
-- local cloud = pcl.io.loadPCDFile('')

function pcl:test()
  local pc = pcl.PointCloud(pcl.PointXYZ)
  pc:loadPCDFile('data/bunny.pcd')
  local t = pc:points()
  print(t)
end

function pcl.test2()
  local s = pcl.OpenNI2Stream()
  local v = pcl.CloudViewer()
  s:start()
  for i=1,10000 do
    local c = s:read(5000)
    if c ~= nil then
      v:showCloud(c)
    end
  end
  s:stop()
end

return pcl