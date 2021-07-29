cmake_minimum_required(VERSION 3.10)

set(GLFW_INCLUDE "C:/Users/WJSSn/Documents/Visual Studio 2017/Libraries/glfw-3.3.2.bin.WIN64/include")
set(GLM_INCLUDE "C:/Users/WJSSn/Documents/Visual Studio 2017/Libraries/glm")

# set the project name
project(Main VERSION 1.0)
find_package(Vulkan REQUIRED)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(MainConfig.h.in MainConfig.h)

# add the executable
add_executable(Main main.cpp)

set(INCLUDE_DIR
  stb_image.h
  tiny_obj_loader.h
  vulkan_window.hpp
  shaders/vert.spv
  shaders/frag.spv
)
include_directories (include/${INCLUDE_DIR})

target_include_directories(Main PUBLIC "${PROJECT_BINARY_DIR}" "${GLFW_INCLUDE}" "${GLM_INCLUDE}" "${VULKAN_INCLUDE}") 
target_link_libraries(Main "C:/Users/WJSSn/Documents/Visual Studio 2017/Libraries/glfw-3.3.2.bin.WIN64/lib-vc2017/glfw3.lib" Vulkan::Vulkan)