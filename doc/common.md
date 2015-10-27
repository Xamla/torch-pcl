# Examples

## Loading a PCD file

```lua
require 'torch-pcl'
local cloud = PointCloud.new(pcl.PointXYZ)
cloud:loadPCDFile('data/bunny.pcd')
local p = cloud:points()	-- get tensor view to points
print(p)
```


