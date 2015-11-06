# Examples

## Loading a PCD file

```lua
local pcl = require 'pcl'
local cloud = pcl.PointCloud(pcl.PointXYZ)
cloud:loadPCDFile('data/bunny.pcd')
local p = cloud:points()  -- get tensor view to points
print(p)
```

