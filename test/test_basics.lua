luaunit = require 'luaunit'
pcl = require 'pcl'

TestBasics = {}

function TestBasics:testAddPointCloud()
  local cloudA = pcl.PointCloud(pcl.PointXYZ,1)
  local cloudB = pcl.PointCloud(pcl.PointXYZ,1)
  cloudA[1]=pcl.PointXYZ(4,5,6)
  cloudB[1]=pcl.PointXYZ(1,2,3)
  cloudA:add(cloudB)
  luaunit.assertEquals(cloudA:height(), 1)
  luaunit.assertEquals(cloudA:width(), 2)
  luaunit.assertEquals(cloudB[1].x, 1)
  print(cloudA:points())
end

os.exit( luaunit.LuaUnit.run() )
