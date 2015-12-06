local ffi = require 'ffi'
local torch = require 'torch'
local utils = require 'pcl.utils'
local pcl = require 'pcl.PointTypes'

local Primitive = {}

local Primitive_method_names = {
  "createSphere",
  "createCube",
  "createCylinder",
  "createCone",
  "createPlatonicSolid",
  "createPlane",
  "createDisk"
}

Primitive.DEFAULT_RESOLUTION = 0.025
Primitive.DEFAULT_SAMPLES = 10000

Primitive.Platonic = {
  TETRAHEDRON   = 0,
  CUBE          = 1,
  OCTAHEDRON    = 2,
  ICOSAHEDRON   = 3,
  DODECAHEDRON  = 4
}
  
local f = utils.create_typed_methods("pcl_Primitive_TYPE_KEY_", Primitive_method_names, 'XYZ')

function Primitive.createSphere(radius, thetaRes, phiRes, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createSphere(output:cdata(), radius or 1, thetaRes or 16, phiRes or 16, 
  samples or Primitive.DEFAULT_SAMPLES, 
  resolution or Primitive.DEFAULT_RESOLUTION)
  return output
end

function Primitive.createCube(x, y, z, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createCube(output:cdata(), x or 1, y or 1, z or 1, 
  samples or Primitive.DEFAULT_SAMPLES, 
  resolution or Primitive.DEFAULT_RESOLUTION);
  return output;
end

function Primitive.createCylinder(height, radius, facets, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createCylinder(output:cdata(), height or 1, radius or 0.5, facets or 32, 
  samples or Primitive.DEFAULT_SAMPLES,
  resolution or Primitive.DEFAULT_RESOLUTION)
  return output
end

function Primitive.createCone(height, radius, facets, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createCone(output:cdata(), height or 1, radius or 0.5, facets or 32, 
  samples or Primitive.DEFAULT_SAMPLES, 
  resolution or Primitive.DEFAULT_RESOLUTION)
  return output
end

function Primitive.createTetrahedron(samples, resolution)
  return Primitive.createPlatonicSolid(Primitive.Platonic.TETRAHEDRON, samples, resolution)
end

function Primitive.createOctahedron(samples, resolution)
  return Primitive.createPlatonicSolid(Primitive.Platonic.OCTAHEDRON, samples, resolution)
end

function Primitive.createIcosahedron(samples, resolution)
  return Primitive.createPlatonicSolid(Primitive.Platonic.ICOSAHEDRON, samples, resolution)
end

function Primitive.createDodecahedron(samples, resolution)
  return Primitive.createPlatonicSolid(Primitive.Platonic.DODECAHEDRON, samples, resolution)
end

function Primitive.createPlatonicSolid(solidType, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createPlatonicSolid(output:cdata(), solidType, 
  samples or Primitive.DEFAULT_SAMPLES, 
  resolution or Primitive.DEFAULT_RESOLUTION)
  return output
end

function Primitive.createPlane(x1, y1, z1, x2, y2, z2, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createPlane(output:cdata(), x1 or 1, y1 or 0, z1 or 0, x2 or 0, y2 or 1, z2 or 0, 
  samples or Primitive.DEFAULT_SAMPLES, 
  resolution or Primitive.DEFAULT_RESOLUTION)
  return output
end

function Primitive.createDisk(innerRadius, outerRadius, radialResolution, samples, resolution)
  local output = pcl.PointCloud('XYZ');
  f.createDisk(output:cdata(), innerRadius or 0, outerRadius or 1, radialResolution or 32, 
  samples or Primitive.DEFAULT_SAMPLES, 
  resolution or Primitive.DEFAULT_RESOLUTION)
  return output
end

pcl.Primitive = Primitive
