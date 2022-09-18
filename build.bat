mkdir build
cd build
conan install .. --build=missing -s build_type=Debug
cmake ..
cmake --build . --config Debug
cd ..