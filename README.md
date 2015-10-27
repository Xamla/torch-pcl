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
$ luarocks install torch-pcl
```

## Usage

```lua
require 'torch-pcl'
c = pcl.PointCloud(pcl.PointXYZ)
c:loadPCDFile('data/bunny.pcd')
p = cloud:points()  -- get tensor view to points
print(p)
```
