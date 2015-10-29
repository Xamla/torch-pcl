local CloudViewer = torch.class('pcl.CloudViewer')

local pcl
local ffi
local func_by_type = {}
local ft = {}

function init(_pcl, _ffi, p, type_key_map)
  pcl = _pcl
  ffi = _ffi
  
  local CloudViewer_method_names = {
    "showCloud"
  }
  
  local generic_names = {}
  for i,n in ipairs(CloudViewer_method_names) do
    generic_names[n] = "pcl_CloudViewer_TYPE_KEY_" .. n
  end
  
  local function create_typed_methods(type_key)
    local map = {}
    for k,v in pairs(generic_names) do
      map[k] = p[string.gsub(v, "TYPE_KEY", type_key)]
    end
    return map
  end
  
  print(type_key_map)
  for k,v in pairs(type_key_map) do
    func_by_type[k] = create_typed_methods(v)
  end
  print(func_by_type)

  ft.new = p["pcl_CloudViewer_new"]
  ft.delete = p["pcl_CloudViewer_delete"]
end

function CloudViewer:__init(window_name)
  self.v = ft.new(window_name)
  ffi.gc(self.v, ft.delete)
end

function CloudViewer:showCloud(cloud, name)
  local f = func_by_type[cloud.pointType];
  if f then
    f.showCloud(self.v, cloud.c, name);
  end
end

return init