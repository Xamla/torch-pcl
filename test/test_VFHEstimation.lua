local pcl = require 'pcl'

x = pcl.rand(1000)

local ne = pcl.NormalEstimation(pcl.PointXYZ)
ne:setKSearch(30)
ne:setInputCloud(x)
local normals = ne:compute()

vf = pcl.VFHEstimation()
vf:setInputCloud(x)
vf:setInputNormals(normals)
out = vf:compute()
print(out[1])
