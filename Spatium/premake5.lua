project "Spatium"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"
    staticruntime "off"
    warnings "Extra"

    location	"" -- Override solution settings.
	targetdir	("../Binaries/Output/" .. BinariesDirectoryFormat .. "/%{prj.name}")
	objdir		("../Binaries/Intermediates/" .. BinariesDirectoryFormat .. "/%{prj.name}")

    files
	{
		"Source/**.h",
		"Source/**.c",
		"Source/**.hpp",
		"Source/**.cpp"
	}

    includedirs
    {
        "Source",
        "%{IncludeDirectories.GLM}",
    }

    filter "configurations:Debug"
        runtime "Debug"
        optimize "Off"
        symbols "On"

    filter "configurations:Release"
        runtime "Release"
        optimize "On"
        symbols "On"