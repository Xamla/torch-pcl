local pcl = require 'pcl'

-- capture a frame from OpenNI device
local s = pcl.OpenNI2Stream()
s:start()


local v = pcl.PCLVisualizer('OrganizedEdges', true)
local edge = pcl.OrganizedEdgeFromNormals()

local ne = pcl.NormalEstimation(pcl.PointXYZ)
ne:setRadiusSearch(0.025)

while not v:wasStopped() do
  c = s:read()
 
  -- run normal estimation
  ne:setInputCloud(c)
  local normals = ne:compute()

  -- detect edges
  edge:setInputCloud(c)
  edge:setInputNormals(normals)
  local labels, label_indices = edge:compute()

  v:removeAllPointClouds()
  for i=1,#label_indices do
    -- extract points with specific edge type
    local y = c:copy(label_indices[i])
    print(i .. ': ' .. label_indices[i]:size())
    v:addPointCloudWithColorHandler(y, pcl.PCLVisualizer.createColorHandlerRandom(y), 'cloud' .. i)
  end
  
  v:spinOnce(100)
end

s:stop()
