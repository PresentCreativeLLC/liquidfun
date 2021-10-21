# Hello World examples
include_directories (${Box2D_SOURCE_DIR})
add_library(UnityPlugin SHARED UnityPlugin.cpp)
target_link_libraries (UnityPlugin Box2D)
