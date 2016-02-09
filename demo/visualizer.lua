local pcl = require 'pcl'

v = pcl.PCLVisualizer('demo', true)
v:addCoordinateSystem()

v:addCube(-2, -1, -2, -1, -1, 1, 0.5, 0.7, 01)
v:addText1('Hi, this is a little PCLVisualizer demo! :-)', 10, 30, 'greetings')
x = v:addPlane_Coefficients({0,0,10,30}, 0, 0, 0)
print(x)

v:addSphere(pcl.PointXYZ(3,3,3), 0.75, 0.1, 0.8, 0.3, 'sphere1')

v:addLine(pcl.PointXYZ(-5,-1,0), pcl.PointXYZ(5, 1, 0), 1, 1, 1, 'line1')

c = pcl.rand(1000)   -- create 1000 random points
v:addPointCloud(c)

v:spin()  -- run msg loop
