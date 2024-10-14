-- Spatium
workspace "Spatium"
    architecture "x64"
    startproject "Spatium"
    location "../"          -- Premake files are generated in the same directory as this script. Hence, redirect.
    
    BinariesDirectoryFormat = "%{cfg.buildcfg}"
    
    configurations
    {
        "Debug",
        "Release"
    }
    
    flags
    {
        "MultiProcessorCompile"
    }
    
    defines
    {
        "NOMINMAX"
    }

include "../Spatium"