local pcl = require 'pcl'

local cloud = pcl.rand(10000)

local removed = pcl.Indices()
local sphere_filtered = pcl.filter.cropSphere(
  cloud,            -- input
  {0.5,0.5,0.5},    -- center
  0.4,              -- radius
  nil,              -- transform
  nil,              -- indices
  nil,              -- output
  false,            -- negative
  removed           -- removed_indices
)

print(string.format('Number of removed point: %d', removed:size()))

local inspector = pcl.PCLVisualizer('Cloud Inspector', true)
inspector:addCoordinateSystem(0.1)

inspector:addPointCloud(cloud, 'cloud')
inspector:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.9, 'cloud')
inspector:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 1, 'cloud')

inspector:addPointCloud(sphere_filtered, 'sphere_filtered')
inspector:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, 'sphere_filtered')
inspector:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, 'sphere_filtered')

print('Please close visualizer window to quit.')
inspector:spin()