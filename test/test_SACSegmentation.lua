local pcl = require 'pcl'

local cloud = pcl.rand(15)
cloud:points()[{{},{},3}] = 1   -- zero out Z compontent

-- set a few outliers
cloud[1].z = 2
cloud[4].z = -2
cloud[7].z = 4

seg = pcl.SACSegmentation('XYZ')
seg:setOptimizeCoefficients(true)

seg:setModelType(pcl.SACMODEL.PLANE)
seg:setMethodType(pcl.SAC.RANSAC)
seg:setDistanceThreshold(0.01)

seg:setInputCloud(cloud)

inliers = pcl.Indices()
coefficients = torch.FloatTensor()

seg:segment(inliers, coefficients)

print(inliers)
print(coefficients)

