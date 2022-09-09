mkdir build
cd build
conan install .. --build=missing
cmake ..
cd ..