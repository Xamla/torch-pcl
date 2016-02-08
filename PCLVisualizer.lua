local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local PCLVisualizer = torch.class('pcl.PCLVisualizer', pcl)

local func = {}

local function init()

  local PCLVisualizermethod_names = {
    'new',
    'delete',
    'delete',
    'setFullScreen',
    'setWindowName',
    'setWindowBorders',
    'spin',
    'spinOnce',
    'addCoordinateSystem',
    'createViewPort',
    'createViewPortCamera',
    'setBackgroundColor',
    'removeAllPointClouds',
    'removeAllShapes',
    'removeAllCoordinateSystems',
    'addText1',
    'addText2',
    'addText3',
    'initCameraParameters',
    'setPointCloudRenderingProperties1',
    'setPointCloudRenderingProperties2',
    'wasStopped',
    'resetStoppedFlag',
    'close'
  }

  func = utils.create_typed_methods("pcl_PCLVisualizer_", PCLVisualizermethod_names,'')
  
  print(func)
end

init()
