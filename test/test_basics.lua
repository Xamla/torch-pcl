luaunit = require 'luaunit'
pcl = require 'pcl'

TestBasics = {}

function TestBasics:testAddPointCloud()
  local cloudA = pcl.PointCloud(pcl.PointXYZ,1)
  local cloudB = pcl.PointCloud(pcl.PointXYZ,1)
  cloudA[1]:set(pcl.PointXYZ(4,5,6))
  luaunit.assertEquals(cloudA[1].x, 4)
  cloudB[1]:set(pcl.PointXYZ(1,2,3))
  luaunit.assertEquals(cloudB[1].x, 1)
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

function TestBasics:testPushInsertErase()
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

function incrementalRegistrationTest(icp)
  -- create 10 'frames'
  local c = pcl.rand(100, 'xyz')
  
  local frames = {}
  for i=1,10 do
    local o = pcl.PointCloud('xyz')
    local T = pcl.affine.translate(i/10,0,0)
    c:transform(T, o)
    table.insert(frames, o)
    c = o
  end
  
  local reg = pcl.IncrementalRegistration()
  reg:setRegistration(icp)
  for i=1,#frames do
    local frame = frames[i]
    local b = reg:registerCloud(frame)
    local t = reg:getDeltaTransform()
    luaunit.assertTrue(b)
    if i > 1 then
      luaunit.assertAlmostEquals(t[{1,4}], -i/10, 0.05)
    end
  end
end

function TestBasics:testIncrementalICP()
  local icp = pcl.ICP()
  icp:setMaxCorrespondenceDistance(0.5)
  icp:setMaximumIterations(50)
  incrementalRegistrationTest(icp)
end

function TestBasics:testIncrementalICPNL()
  local icpnl = pcl.ICPNL()
  icpnl:setMaximumIterations(50)
  icpnl:setEuclideanFitnessEpsilon(0.001)
  
  incrementalRegistrationTest(icpnl)
end

function TestBasics:testICPNLWithNormals()
  local icp = pcl.ICPNL(pcl.PointNormal)
  icp:setMaximumIterations(100)
  icp:setEuclideanFitnessEpsilon(0.000001)

  -- generate 2x 500 random points on plane
  local g = pcl.primitive.plane(1,0,0, 0,0,1, 500, 0.01)
  local h = pcl.primitive.plane(1,0,0, 0,0,1, 500, 0.01)
  h:transform(pcl.affine.translate(-0.5,-0.75,-0.5))

  local ne = pcl.NormalEstimation()
  ne:setRadiusSearch(0.1)

  -- estimate normals for h
  ne:setInputCloud(g)
  local gn = ne:compute()
  g = g:addNormals(gn)

  -- estimate normals for g
  ne:setInputCloud(h)
  local hn = ne:compute()
  h = h:addNormals(hn)

  icp:setInputSource(g)
  icp:setInputTarget(h)

  icp:align()
  local finalTransformation = icp:getFinalTransformation()
  luaunit.assertAlmostEquals(finalTransformation[{2,4}], -0.75, 0.05)
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

function TestBasics:testVoxelHistogram()
  local p = pcl.rand(1000)
  local o,c = pcl.filter.voxelHistogram(p, 1, 1, 1, 1)
  local s = o:size()
  luaunit.assertEquals(c, 1000)
  luaunit.assertAlmostEquals(o:sum(), 1000, 0.01)
  luaunit.assertTrue(s[1] == 1 and s[2] == 1 and s[3] == 1)
  
  local o,c = pcl.filter.voxelHistogram(p, 5, 5, 5, 1.0/5, {0,0,0}, true)
  luaunit.assertAlmostEquals(o:sum(), 1000/8, 50)
  luaunit.assertAlmostEquals(o:sum(), c, 0.1)
  luaunit.assertAlmostEquals(o[{{4,5},{},{}}]:sum(), 0, 0.1)
  luaunit.assertTrue(o:max() < 30)
end

local function sqaure_dist(a,b)
  return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)
end
  
function TestBasics:testKdTreeSearch()
  local cloud = pcl.rand(1000)
  local kd = pcl.KdTree()
  kd:setInputCloud(cloud)
  
  local indices = pcl.Indices()
  local squaredDistances = torch.FloatTensor()
  
  local p = pcl.PointXYZ(0.5, 0.5, 0.5)
  kd:nearestKSearch(p, 10, indices, squaredDistances)
  luaunit.assertEquals(10, indices:size())
  luaunit.assertEquals(10, squaredDistances:size(1))
  for i=1,10 do
    luaunit.assertAlmostEquals(sqaure_dist(cloud[indices[i]], p), squaredDistances[i], 0.01)
  end
  luaunit.assertTrue(squaredDistances:mean() < 0.1)
  
  local found = kd:radiusSearch(p, 0.2, indices, squaredDistances)
  luaunit.assertEquals(found, indices:size())
  luaunit.assertTrue(found > 0)
  for i=1,10 do
    luaunit.assertAlmostEquals(sqaure_dist(cloud[indices[i]], p), squaredDistances[i], 0.01)
    luaunit.assertTrue(sqaure_dist(cloud[indices[i]], p) < 0.201*0.201)
  end
end

function TestBasics:testOctreeSearch()
  local cloud = pcl.rand(1000)
  local ot = pcl.Octree(0.001)
  ot:setInputCloud(cloud)
  ot:addPointsFromInputCloud()
  
  luaunit.assertTrue(ot:getLeafCount() > 950)
  
  local indices = pcl.Indices()
  local squaredDistances = torch.FloatTensor()
  
  local p = pcl.PointXYZ(0.5, 0.5, 0.5)
  ot:nearestKSearch(p, 10, indices, squaredDistances)
  luaunit.assertEquals(10, indices:size())
  luaunit.assertEquals(10, squaredDistances:size(1))
  for i=1,10 do
    luaunit.assertAlmostEquals(sqaure_dist(cloud[indices[i]], p), squaredDistances[i], 0.01)
  end
  luaunit.assertTrue(squaredDistances:mean() < 0.1)
  
  local found = ot:radiusSearch(p, 0.2, indices, squaredDistances)
  luaunit.assertEquals(found, #indices)
  luaunit.assertTrue(found > 0)
  for i=1,10 do
    luaunit.assertAlmostEquals(sqaure_dist(cloud[indices[i]], p), squaredDistances[i], 0.01)
    luaunit.assertTrue(sqaure_dist(cloud[indices[i]], p) < 0.201*0.201)
  end
end

function TestBasics:testCovariance()
  local cloud = pcl.rand(1000)
  local centroid = cloud:compute3DCentroid()
  luaunit.assertAlmostEquals(centroid[{{1,3}}]:norm(), 0.86, 0.1)
  
  local cov = cloud:computeCovarianceMatrix(centroid)
  for i=1,3 do
    luaunit.assertTrue(cov[{i,i}] > 50)
  end
end

function TestBasics:testNormalEstimation()
  -- create plane
  local g = pcl.primitive.plane(1,0,0, 0,0,1, 500, 0.01)
  g:transform(pcl.affine.translate(-0.5,-1,-0.5))
  
  local kd = pcl.KdTree()
  kd:setInputCloud(g)
  
  local ne = pcl.NormalEstimation()
  ne:setRadiusSearch(0.1)
  ne:setSearchMethod(kd)
  ne:setInputCloud(g)
  local out = ne:compute()
  for i=1,#out do
    luaunit.assertAlmostEquals(out[i].normal_x, 0, 0.1)
    luaunit.assertAlmostEquals(out[i].normal_y, 1, 0.1)
    luaunit.assertAlmostEquals(out[i].normal_z, 0, 0.1)
  end
end

function TestBasics:testIndices()
  local idx = pcl.Indices()
  for i=1,100 do
    idx:push_back(i)
  end
  luaunit.assertEquals(idx:size(), 100)
  luaunit.assertEquals(#idx, 100)
  idx:erase(1,51)
  luaunit.assertEquals(#idx, 50)
  luaunit.assertEquals(idx[1], 51)
  
  local t = idx:viewAsTensor()
  for i=51,100 do
    luaunit.assertEquals(t[i-50], i-1)
  end
  
  local idx2 = pcl.Indices({5,7,9})
  luaunit.assertEquals(#idx2, 3)
  luaunit.assertEquals(idx2[1], 5)
  luaunit.assertEquals(idx2[2], 7)
  luaunit.assertEquals(idx2[3], 9)
  
  idx2:append(idx)
  luaunit.assertEquals(#idx2, 53)
  luaunit.assertEquals(idx2[53], 100)
end

function TestBasics:testCopyPointCloud()
  local a = pcl.PointCloud('xyzi', 20)
  for i=1,20 do
    a[i].x = i*1
    a[i].y = i*2
    a[i].z = i*3
    a[i].intensity = i*4
  end
  local b = pcl.PointCloud('xyzrgba', 10)
  a:copy(b)
  luaunit.assertEquals(b:size(), 20)
  luaunit.assertEquals(a.pointType, pcl.PointXYZI)
  luaunit.assertEquals(b.pointType, pcl.PointXYZRGBA)
  for i=1,10 do
    luaunit.assertEquals(b[i].x, i*1)
    luaunit.assertEquals(b[i].y, i*2)
    luaunit.assertEquals(b[i].z, i*3)
  end
end

os.exit( luaunit.LuaUnit.run() )
