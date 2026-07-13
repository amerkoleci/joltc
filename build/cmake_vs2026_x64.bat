@echo off
cmake -S "./../" -B "vs2026_x64" -G "Visual Studio 18 2026" -A x64 -DCMAKE_INSTALL_PREFIX:String="SDK" -DCMAKE_BUILD_TYPE:String=Distribution %*
echo Open vs2026_x64\JoltC.sln to build the project.