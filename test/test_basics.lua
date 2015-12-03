luaunit = require 'luaunit'
pcl = require 'pcl'

TestBasics = {}

function TestBasics:testAddPointCloud()
  local cloudA = pcl.PointCloud(pcl.PointXYZ,1)
  local cloudB = pcl.PointCloud(pcl.PointXYZ,1)
  cloudA[1]:set(pcl.PointXYZ(4,5,6))
  cloudB[1]:set(pcl.PointXYZ(1,2,3))
  cloudA:add(cloudB)
  luaunit.assertEquals(cloudA:height(), 1)
  luaunit.assertEquals(cloudA:width(), 2)
  luaunit.assertEquals(cloudB[1].x, 1)
end

function TestBasics:testPointTypeString()
  local c1 = pcl.PointCloud('xyz')
  local c2 = pcl.PointCloud('xYZi')
  local c3 = pcl.PointCloud('PointXYZRGBA')
  luaunit.assertEquals(c1.pointType, pcl.PointXYZ)
  luaunit.assertEquals(c2.pointType, pcl.PointXYZI)
  luaunit.assertEquals(c3.pointType, pcl.PointXYZRGBA)
  luaunit.assertNotEquals(c3.pointType, pcl.PointXYZ)
end

function TestBasics:testFromTensor()
  local tf = torch.FloatTensor({{ 4,3,2,1 }, { 5,3,2,1 }})
  local td = torch.DoubleTensor({{ 5,9,7,2 }, { 1,-3,7,0 }})
  local pc = pcl.PointCloud(pcl.PointXYZ, tf)
  pc:add(pcl.PointCloud('xyz', td))
  local p = pc:points()
  luaunit.assertAlmostEquals((p - torch.FloatTensor({{ 4,3,2,1 }, { 5,3,2,1 }, { 5,9,7,2 }, { 1,-3,7,0 }})):abs():sum(), 0, 0.001)
end

function TestBasics:testLoadPCD()
  local pc = pcl.PointCloud(pcl.PointXYZ)
  luaunit.assertTrue(pc:empty())
  pc:loadPCDFile('../data/bunny.pcd')
  luaunit.assertEquals(pc:width(), 397)
end

function TestBasics:testPCA()
  local pca = pcl.PCA(pcl.rand(1000, pcl.PointXYZ))
  luaunit.assertAlmostEquals(pca:get_mean()[{{1,3}}]:mean(), 0.5, 0.2)
end

os.exit( luaunit.LuaUnit.run() )
