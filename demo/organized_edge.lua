local pcl = require 'pcl'

-- capture a frames from OpenNI device
local s = pcl.OpenNI2Stream()
s:start()

-- use PCLVisualizer to display edge point clouds in different colors
local v = pcl.PCLVisualizer('OrganizedEdges', true)

local colors = {
  pcl.COLORS.WHITE,
  pcl.COLORS.RED,
  pcl.COLORS.GREEN,
  pcl.COLORS.BLUE,
  pcl.COLORS.PURPLE,
}

function visualize(cloud, label_indices)
  v:removeAllPointClouds()
  v:addPointCloud(cloud, 'base')
  v:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 0.2, 0.2, 0.2, 'base')
  for i=1,#label_indices do
    local color = colors[i]
    local id = 'cloud' .. i
    local y = cloud:copy(label_indices[i])    -- extract points with specific edge type
    print(i .. ': ' .. label_indices[i]:size())
    v:addPointCloud(y, id)
    v:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, color[1], color[2], color[3], id)
  end
end
  
function RunBase()
  local edge = pcl.OrganizedEdgeBase()
  edge:setEdgeType(bit.bor(pcl.EDGELABEL.OCCLUDING))
  edge:setEdgeType(bit.bor(pcl.EDGELABEL.OCCLUDING))
  edge:setDepthDisconThreshold(0.03)

  while not v:wasStopped() do
    c = s:read()
    edge:setInputCloud(c)
    local labels, label_indices = edge:compute()
    visualize(c, label_indices)
    v:spinOnce(100)
  end

  edge:setInputCloud(c)
end

function RunWithNormals()
  local edge = pcl.OrganizedEdgeFromNormals()
  edge:setEdgeType(bit.bor(pcl.EDGELABEL.NAN_BOUNDARY, pcl.EDGELABEL.OCCLUDING, pcl.EDGELABEL.OCCLUDED, pcl.EDGELABEL.HIGH_CURVATURE))
  edge:setDepthDisconThreshold(0.03)
  edge:setHCCannyLowThreshold(0.75)
  edge:setHCCannyHighThreshold(1.5)

  local ne = pcl.IntegralImageNormalEstimation(pcl.PointXYZ)

  while not v:wasStopped() do

    c = s:read()
    -- run normal estimation
    ne:setInputCloud(c)
    local normals = ne:compute()

    -- detect edges
    edge:setInputCloud(c)
    edge:setInputNormals(normals)
    local labels, label_indices = edge:compute()
    visualize(c, label_indices)
    v:spinOnce(100)
  end

end

--RunBase()
RunWithNormals()

s:stop()
