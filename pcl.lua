local ffi = require 'ffi'

function libPath(libName)
    if     ffi.os == 'Windows' then
        return 'lib/' .. libName .. '.dll'
    elseif ffi.os == 'OSX' then
        return 'lib/lib' .. libName .. '.dylib'
    else
        return 'lib/lib' .. libName .. '.so'
    end
end

ffi.cdef[[

]]

local C = ffi.load(libPath('Common'))

pcl = {}

return pcl