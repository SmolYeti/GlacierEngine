include(./.env.cmake OPTIONAL RESULT_VARIABLE LOCAL_ENV)
message(STATUS "Local .env.cmake: ${LOCAL_ENV}")

cmake_minimum_required(VERSION 3.11.0)

set(NAME GlacierEngine)
 
message(STATUS "using ${CMAKE_GENERATOR}")

# set the project name 
project(${NAME} VERSION 0.0.01)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

# 1. Set VULKAN_SDK_PATH in .env.cmake to target specific vulkan version
if (DEFINED VULKAN_SDK_PATH)
  set(Vulkan_INCLUDE_DIRS "${VULKAN_SDK_PATH}/Include") # 1.1 Make sure this include path is correct
  set(Vulkan_LIBRARIES "${VULKAN_SDK_PATH}/Lib") # 1.2 Make sure lib path is correct
  set(Vulkan_FOUND "True")
else()
  find_package(Vulkan REQUIRED) # throws error if could not find Vulkan
  message(STATUS "Found Vulkan: $ENV{VULKAN_SDK}")
endif()
if (NOT Vulkan_FOUND)
    message(FATAL_ERROR "Could not find Vulkan library!")
else()
    message(STATUS "Using vulkan lib at: ${Vulkan_LIBRARIES}")
endif()

include_directories(external)

# If TINYOBJ_PATH not specified in .env.cmake, try fetching from git repo
if (NOT TINYOBJ_PATH)
  message(STATUS "TINYOBJ_PATH not specified in .env.cmake, using external/tinyobjloader")
  set(TINYOBJ_PATH external/tinyobjloader)
endif()

configure_file(MainConfig.h.in MainConfig.h)

# Add NURBS Library
add_subdirectory(nurbs_cpp)

file(GLOB_RECURSE VULKENG_DIR
  ${PROJECT_SOURCE_DIR}/vulkeng/*.cpp
  ${PROJECT_SOURCE_DIR}/vulkeng/*.hpp
)

file(GLOB_RECURSE IMGUI_DIR
  ${PROJECT_SOURCE_DIR}/bindings/*.hpp
  ${PROJECT_SOURCE_DIR}/bindings/*.cpp
)

file(GLOB_RECURSE GLSL_SOURCE_FILES
  "${PROJECT_SOURCE_DIR}/shaders/*.frag"
  "${PROJECT_SOURCE_DIR}/shaders/*.vert"
)

add_executable(${PROJECT_NAME} main.cpp ${VULKENG_DIR} ${IMGUI_DIR} ${GLSL_SOURCE_FILES})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/build")

if (WIN32)
  message(STATUS "CREATING BUILD FOR WINDOWS")

  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/nurbs_cpp
    ${PROJECT_SOURCE_DIR}/bindings
    ${Vulkan_INCLUDE_DIRS}
    ${TINYOBJ_PATH}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_PATH}
  )

  target_link_directories(${PROJECT_NAME} PUBLIC ${GLFW_LIB} ${Vulkan_LIBRARIES})

  target_link_libraries(${PROJECT_NAME} glfw3 vulkan-1 nurbs_cpp imgui)
elseif (UNIX)
    message(STATUS "CREATING BUILD FOR UNIX")
    target_include_directories(${PROJECT_NAME}
    PUBLIC
      ${PROJECT_SOURCE_DIR}
      ${TINYOBJ_PATH}
    )
    target_link_libraries(${PROJECT_NAME} glfw ${Vulkan_LIBRARIES} imgui)
endif()

############## Build SHADERS #######################
 
# Find all vertex and fragment sources within shaders directory
# taken from VBlancos vulkan tutorial
# https://github.com/vblanco20-1/vulkan-guide/blob/all-chapters/CMakeLists.txt
find_program(GLSL_VALIDATOR glslangValidator HINTS /usr/bin /usr/local/bin $ENV{VULKAN_SDK}/Bin/ $ENV{VULKAN_SDK}/Bin32/)

foreach(GLSL ${GLSL_SOURCE_FILES})
  message(STATUS "BUILDING SHADER")
  get_filename_component(FILE_NAME ${GLSL} NAME)
  set(SPIRV "${PROJECT_SOURCE_DIR}/shaders/${FILE_NAME}.spv")
  message(STATUS ${GLSL})
  add_custom_command(
    OUTPUT ${SPIRV}
    COMMAND ${GLSL_VALIDATOR} -V ${GLSL} -o ${SPIRV}
    DEPENDS ${GLSL})
  list(APPEND SPIRV_BINARY_FILES ${SPIRV})
endforeach(GLSL)

add_custom_target(
  Shaders 
  DEPENDS ${SPIRV_BINARY_FILES}
)

add_dependencies(${NAME} Shaders)

enable_testing()

# nc1 => NURBS Chapter 1
add_executable(nurbs_tests
  tests/nc1_bezier_tests.cpp
  tests/nc1_parametric_tests.cpp
  tests/nc1_power_basis_tests.cpp
  tests/nc2_basis_tests.cpp
  tests/nc3_b_spline_tests.cpp
)

target_include_directories(nurbs_tests PUBLIC
  ${PROJECT_SOURCE_DIR}/nurbs_cpp
)

target_link_libraries(nurbs_tests
    ${CONAN_LIBS}
    nurbs_cpp
)

include(GoogleTest)
gtest_discover_tests(nurbs_tests)