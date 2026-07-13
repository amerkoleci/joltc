@echo off
cmake -B "vs2026_arm64" -S "./../" -G "Visual Studio 18 2026" -A ARM64 -DCMAKE_INSTALL_PREFIX:String="Sdk" %*
echo Open vs2026_arm64\JoltC.sln to build the project.