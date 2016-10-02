local pcl = require 'pcl'

local cloud = pcl.rand(10000)

local removed = pcl.Indices()

local box_filtered = pcl.filter.cropBox(
  cloud,        -- input
  {0.25,0.25,0.25},  -- min
  {0.75,0.75,0.75},  -- max
  nil,          -- rotation
  nil,          -- translation
  pcl.affine.translate(0.5, 0.5, 0.5) * pcl.affine.rotateAxis({1,1,1}, 0.25 * math.pi) * pcl.affine.translate(-0.5, -0.5, -0.5),   -- transform
  nil,          -- indices
  nil,          -- output
  false,        -- negative
  removed       -- removed_indices
)

print(string.format('Number of removed points: %d', removed:size()))

-- visualize result
local inspector = pcl.PCLVisualizer('Cloud Inspector', true)
inspector:addCoordinateSystem(0.1)

inspector:addPointCloud(cloud, 'cloud')
inspector:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.9, 'cloud')
inspector:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 1, 'cloud')

inspector:addPointCloud(box_filtered, 'box_filtered')
inspector:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, 'box_filtered')
inspector:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, 'box_filtered')

print('Please close visualizer window to quit.')
inspector:spin()
