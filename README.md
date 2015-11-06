# torch-pcl
Point Cloud Library (PCL) bindings for Torch

*WARNING*: Work in progress! Expected v1 release date: 2015-12-20
If you want to help please contact [@andreaskoepf](https://github.com/andreaskoepf).

## Install

Prerequisites:

1. [Torch7](http://torch.ch/docs/getting-started.html)
2. [PCL](http://pointclouds.org/downloads/linux.html)

Install using LuaRocks:

```bash
$ luarocks install pcl
```

## Some Examples

### Load a PCD file and acess values as torch tensor.

```lua
local pcl = require 'pcl'
cloud = pcl.PointCloud(pcl.PointXYZ)
cloud:loadPCDFile('data/bunny.pcd')
pt = cloud:points()  -- get tensor view to points
print(pt)
```

### Visualize live RGB-D sensor data 

Capture cloud point with OpenNI2 device and show result live in a cloud viewer window.

```lua
local pcl = require 'pcl'
local s = pcl.OpenNI2Stream()
local v = pcl.CloudViewer()
s:start()
for i=1,1000 do
  local c = s:read(1000)
  if c ~= nil then
    v:showCloud(c)
  end
end
s:stop()
```
