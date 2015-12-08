local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local primitive = {}

local primitive_method_names = {
  "createSphere",
  "createCube",
  "createCylinder",
  "createCone",
  "createPlatonicSolid",
  "createPlane",
  "createDisk"
}

primitive.DEFAULT_RESOLUTION = 0.025
primitive.DEFAULT_SAMPLES = 10000

primitive.Platonic = {
  TETRAHEDRON   = 0,
  CUBE          = 1,
  OCTAHEDRON    = 2,
  ICOSAHEDRON   = 3,
  DODECAHEDRON  = 4
}
  
local f = utils.create_typed_methods("pcl_Primitive_TYPE_KEY_", primitive_method_names, 'XYZ')

function primitive.sphere(radius, thetaRes, phiRes, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createSphere(output:cdata(), radius or 1, thetaRes or 16, phiRes or 16, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION)
  return output
end

function primitive.cube(x, y, z, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createCube(output:cdata(), x or 1, y or 1, z or 1, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION);
  return output;
end

function primitive.cylinder(height, radius, facets, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createCylinder(output:cdata(), height or 1, radius or 0.5, facets or 32, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION)
  return output
end

function primitive.cone(height, radius, facets, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createCone(output:cdata(), height or 1, radius or 0.5, facets or 32, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION)
  return output
end

function primitive.tetrahedron(samples, resolution)
  return primitive.platonicSolid(primitive.Platonic.TETRAHEDRON, samples, resolution)
end

function primitive.octahedron(samples, resolution)
  return primitive.platonicSolid(primitive.Platonic.OCTAHEDRON, samples, resolution)
end

function primitive.icosahedron(samples, resolution)
  return primitive.platonicSolid(primitive.Platonic.ICOSAHEDRON, samples, resolution)
end

function primitive.dodecahedron(samples, resolution)
  return primitive.platonicSolid(primitive.Platonic.DODECAHEDRON, samples, resolution)
end

function primitive.platonicSolid(solidType, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createPlatonicSolid(output:cdata(), solidType, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION)
  return output
end

function primitive.plane(x1, y1, z1, x2, y2, z2, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createPlane(output:cdata(), x1 or 1, y1 or 0, z1 or 0, x2 or 0, y2 or 1, z2 or 0, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION)
  return output
end

function primitive.disk(innerRadius, outerRadius, radialResolution, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createDisk(output:cdata(), innerRadius or 0, outerRadius or 1, radialResolution or 32, 
  samples or primitive.DEFAULT_SAMPLES,
  resolution or primitive.DEFAULT_RESOLUTION)
  return output
end

pcl.primitive = primitive
