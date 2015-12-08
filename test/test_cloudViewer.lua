pcl = require 'pcl'

function test_wasStopped()
  local cloud=pcl.PointCloud(pcl.PointXYZI,20000) -- cloud with width=100, height=100
  local viewer=pcl.CloudViewer('viewer1')          -- cloud viewer with name 'viewer1'

  cloud:points():normal(0, 1)
  viewer:showCloud(cloud)
  
  repeat
  until viewer:wasStopped()
end

function test_OpenNI2Stream()
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

test_wasStopped()