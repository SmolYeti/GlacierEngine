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
#add_executable(Main main.cpp)

# TODO - Move Vulkeng imports into their own library and include library here (static linking, not dlls)
set(INCLUDE_DIR
  vulkeng/include/simple_render_system.hpp
  include/stb_image.h
  include/tiny_obj_loader.h
  vulkeng/include/vulkan_application.hpp
  vulkeng/include/vulkan_device.hpp
  vulkeng/include/vulkan_game_object.hpp
  vulkeng/include/vulkan_model.hpp
  vulkeng/include/vulkan_pipeline.hpp
  vulkeng/include/vulkan_renderer.hpp
  vulkeng/include/vulkan_swap_chain.hpp
  vulkeng/include/vulkan_window.hpp
  shaders/vert.spv
  shaders/frag.spv
)

set(SOURCE_DIR
  main.cpp
  vulkeng/src/simple_render_system.cpp
  vulkeng/src/vulkan_application.cpp
  vulkeng/src/vulkan_device.cpp
  vulkeng/src/vulkan_model.cpp
  vulkeng/src/vulkan_pipeline.cpp
  vulkeng/src/vulkan_renderer.cpp
  vulkeng/src/vulkan_swap_chain.cpp
  vulkeng/src/vulkan_window.cpp
)

add_executable(Main ${INCLUDE_DIR} ${SOURCE_DIR})

target_include_directories(Main PUBLIC "${PROJECT_BINARY_DIR}" "${GLFW_INCLUDE}" "${GLM_INCLUDE}" "${VULKAN_INCLUDE}") 
target_link_libraries(Main "C:/Users/WJSSn/Documents/Visual Studio 2017/Libraries/glfw-3.3.2.bin.WIN64/lib-vc2017/glfw3.lib" Vulkan::Vulkan)


# From:
# https://stackoverflow.com/questions/57478571/why-wont-cmake-build-my-vulkan-spirv-shaders

find_package(Vulkan REQUIRED COMPONENTS glslc)
find_program(glslc_executable NAMES glslc HINTS Vulkan::glslc)

#==============================================================================
# COMPILE SHADERS
#

set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/shaders)

file(GLOB SHADERS_FILES
  ${SHADER_SOURCE_DIR}/*.vert
  ${SHADER_SOURCE_DIR}/*.frag
  ${SHADER_SOURCE_DIR}/*.comp
  ${SHADER_SOURCE_DIR}/*.geom
  ${SHADER_SOURCE_DIR}/*.tesc
  ${SHADER_SOURCE_DIR}/*.tese
  ${SHADER_SOURCE_DIR}/*.mesh
  ${SHADER_SOURCE_DIR}/*.task
  ${SHADER_SOURCE_DIR}/*.rgen
  ${SHADER_SOURCE_DIR}/*.rchit
  ${SHADER_SOURCE_DIR}/*.rmiss)

add_custom_command(
  COMMAND
    ${CMAKE_COMMAND} -E make_directory ${SHADER_BINARY_DIR}
  OUTPUT ${SHADER_BINARY_DIR}
  COMMENT "Creating ${SHADER_BINARY_DIR}"
)

foreach(source IN LISTS SHADERS_FILES)
  get_filename_component(FILENAME ${source} NAME)
  add_custom_command(
    COMMAND
      ${glslc_executable}
      #      -MD -MF ${SHADER_BINARY_DIR}/${FILENAME}.d
      -o ${SHADER_BINARY_DIR}/${FILENAME}.spv
      ${source}
    OUTPUT ${SHADER_BINARY_DIR}/${FILENAME}.spv
    DEPENDS ${source} ${SHADER_BINARY_DIR}
    COMMENT "Compiling ${FILENAME}"
  )
  list(APPEND SPV_SHADERS ${SHADER_BINARY_DIR}/${FILENAME}.spv)
endforeach()

add_custom_target(shaders_defs ALL DEPENDS ${SPV_SHADERS})
