luaunit = require 'luaunit'
pcl = require 'pcl'

TestData = {}

  function TestData:testReadRGBAbyte()
    a = pcl.PointCloud(pcl.PointXYZRGBA, 10, 10)
    a[1].r = 1
    a[1].g = 2
    a[1].b = 3
    o = torch.ByteTensor()
    a:readRGBA(o)
    print(o[{1,1,{}}])
    print(o:size())
    luaunit.assertEquals(torch.type(o), "torch.ByteTensor")
  end

os.exit( luaunit.LuaUnit.run() )
