@echo off
cmake -S "./../" -B "vs2026_x64_double" -G "Visual Studio 18 2026" -A x64 -DCMAKE_INSTALL_PREFIX:String="SDK" -DDOUBLE_PRECISION=ON %*
echo Open vs2026_x64_double\JoltC.sln to build the project.