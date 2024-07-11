file(GLOB_RECURSE NURBS_DIR
  ${PROJECT_SOURCE_DIR}/nurbs_cpp/*.cpp
  ${PROJECT_SOURCE_DIR}/nurbs_cpp/*.hpp
)

add_library(nurbs_cpp ${NURBS_DIR})

target_include_directories(nurbs_cpp PUBLIC
    ${PROJECT_SOURCE_DIR}/nurbs_cpp
)