luaunit = require 'luaunit'
pcl = require 'pcl'

TestBasics = {}

function TestBasics:testAddPointCloud()
  local cloudA = pcl.PointCloud(pcl.PointXYZ,1)
  local cloudB = pcl.PointCloud(pcl.PointXYZ,1)
  cloudA[1]:set(pcl.PointXYZ(4,5,6))
  cloudB[1]:set(pcl.PointXYZ(1,2,3))
  cloudA:add(cloudB)
  luaunit.assertEquals(cloudA:getHeight(), 1)
  luaunit.assertEquals(cloudA:getWidth(), 2)
  luaunit.assertEquals(cloudA:size(), 2)
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

function TestBasics:testPointType()
  local x = pcl.PointXYZ():set({3,2,1})
  luaunit.assertTrue(x == {3,2,1})
  luaunit.assertEquals(x, pcl.PointXYZ(3,2,1))
  local y = pcl.PointXYZ():set({99,99,99})
  luaunit.assertEquals(y, pcl.PointXYZ(99,99,99))
  luaunit.assertNotEquals(y, pcl.PointXYZ(99,99,98))
  
  local z = pcl.PointXYZRGBA(1,2,3,0xaabbccdd)
  luaunit.assertEquals(z.rgba, 0xaabbccdd)
  luaunit.assertEquals(z.a, 0xaa)
  
  local w = pcl.PointXYZI(7,6,5,123.5)
  luaunit.assertTrue(w == {7,6,5,0,123.5})
  
  -- smaller point to the right
  luaunit.assertTrue(pcl.PointXYZI(1,2,3,4) == pcl.PointXYZ(1,2,3))
  luaunit.assertFalse(pcl.PointXYZI(1,2,3,4) == pcl.PointXYZ(1,2,4))
  luaunit.assertTrue(pcl.PointXYZRGBA(1,2,3) == pcl.PointXYZ(1,2,3))
end

function TestBasics:testPushBackInsertErase()
  local c = pcl.PointCloud('xyz')
  for i=1,10 do
    c:push_back(pcl.PointXYZ(i,i+1,i+2))
  end
  luaunit.assertEquals(c:size(), 10)
  c:erase(1)
  luaunit.assertEquals(c:size(), 9)
  luaunit.assertEquals(c[1].x, 2)
  luaunit.assertEquals(c[1].y, 3)
  luaunit.assertEquals(c[1][3], 4)
  c:erase(2, 5)
  luaunit.assertEquals(c[2][1], 6)
  c:insert(3, pcl.PointXYZ({8,7,5}), 100)
  luaunit.assertEquals(c:size(), 106)
  luaunit.assertTrue(c[3] == pcl.PointXYZ(8,7,5))
  luaunit.assertTrue(c[102] == pcl.PointXYZ(8,7,5))
  luaunit.assertFalse(c[103] == pcl.PointXYZ(8,7,5))
  c:insert(1, pcl.PointXYZ(-1,-2,-3))
  luaunit.assertTrue(c[1] == {-1, -2, -3})
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
  luaunit.assertEquals(pc:getWidth(), 397)
end

function TestBasics:testPCA()
  local c = pcl.rand(1000, pcl.PointXYZ)
  
  local pca = pcl.PCA()
  pca:setInputCloud(c)
  luaunit.assertAlmostEquals(pca:getMean()[{{1,3}}]:mean(), 0.5, 0.2)
  
  local pca2 = c:pca()
  luaunit.assertAlmostEquals(pca:getMean()[{{1,3}}]:mean(), pca2:getMean()[{{1,3}}]:mean(), 0.2)
end

function TestBasics:testICP()
  local cloudIn = pcl.rand(10, 10, pcl.PointXYZ)
  local cloudOut = pcl.PointCloud(pcl.PointXYZ)
  local T = pcl.affine.translate(0.5,0,0)
  cloudIn:transform(T, cloudOut)
  luaunit.assertTrue((cloudIn:points() - cloudOut:points()):abs():sum() > 1)
  icp = pcl.ICP()
  icp:setInputSource(cloudIn)
  icp:setInputTarget(cloudOut)
  local cloudFinal = icp:align(cloudFinal)
  local finalTransformation = icp:getFinalTransformation()
  luaunit.assertTrue((finalTransformation - T):abs():sum() < 0.1)
  luaunit.assertAlmostEquals(icp:getFitnessScore(), 0, 0.01)
end

function TestBasics:testFilterDefaultValues()
  local p = pcl.rand(100, 100)
  
  pcl.filter.cropBox(p)
  pcl.filter.cropSphere(p)
  pcl.filter.medianFilter(p)
  pcl.filter.statisticalOutlierRemoval(p)
  --pcl.filter.radiusOutlierRemoval(p)
  pcl.filter.passThrough(p, 'x')
  pcl.filter.randomSample(p)
  pcl.filter.voxelGrid(p)
  
end

function TestBasics:testNaNRemoval()
  local p = pcl.rand(100)
  luaunit.assertEquals(p:size(), 100)
  
  p[1].x = 0/0
  p:setIsDense(false)
  luaunit.assertFalse(p:getIsDense())
  
  -- inplace first
  p:removeNaN()
  luaunit.assertEquals(p:size(), 99)
  luaunit.assertTrue(p:getIsDense())
  
  p[1].x = 0/0
  p:setIsDense(false)
  luaunit.assertFalse(p:getIsDense())
  
  local p2 = pcl.filter.removeNaNFromPointCloud(p)
  luaunit.assertEquals(p:size(), 99)
  luaunit.assertFalse(p:getIsDense())
  luaunit.assertEquals(p2:size(), 98)
  luaunit.assertTrue(p2:getIsDense())
end

os.exit( luaunit.LuaUnit.run() )
