# Examples

## Loading a PCD file

```lua
require 'torch-pcl'
local cloud = pcl.PointCloud(pcl.PointXYZ)
cloud:loadPCDFile('data/bunny.pcd')
local p = cloud:points()  -- get tensor view to points
print(p)
```

