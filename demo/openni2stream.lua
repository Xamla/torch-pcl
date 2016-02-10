pcl = require 'pcl'

v = pcl.CloudViewer()
s = pcl.OpenNI2Stream()

s:start()

while true do
  local c = s:read()
  v:showCloud(c)
  collectgarbage()
end
