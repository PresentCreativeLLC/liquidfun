# Hello World examples
include_directories (${Box2D_SOURCE_DIR})
add_library(UnityPlugin SHARED UnityPlugin.cpp)
target_link_libraries (UnityPlugin Box2D)

set_target_properties(UnityPlugin PROPERTIES
		OUTPUT_NAME "Liquidfun_UnityPlugin"
		CLEAN_DIRECT_OUTPUT 1
		VERSION ${BOX2D_VERSION}
	)
