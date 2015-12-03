pcl = require 'pcl'

function test_wasStopped()
  local cloud=pcl.PointCloud(pcl.PointXYZI,20000) -- cloud with width=100, height=100
  local viewer=pcl.CloudViewer('viewer1')          -- cloud viewer with name 'viewer1'

  cloud:points():normal(0, 1)
  viewer:showCloud(cloud)
  
  repeat
  until viewer:wasStopped()
end

test_wasStopped()