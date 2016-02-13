package = "torch-pcl"
version = "scm-1"

source = {
   url = "git@github.com:Xamla/torch-pcl.git",
}

description = {
   summary = "Point Cloud Library (PCL) bindings for Torch",
   detailed = [[
   ]],
   homepage = "https://github.com/Xamla/torch-pcl",
   license = "BSD"
}

dependencies = {
   "torch >= 7.0"
}

build = {
   type = "command",
   build_command = [[
cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(LUA_BINDIR)/.." -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE) -j 8
]],
   install_command = "cd build && $(MAKE) install"
}
