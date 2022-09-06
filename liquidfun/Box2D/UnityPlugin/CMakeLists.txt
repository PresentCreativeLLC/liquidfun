# Hello World examples
include_directories (${Box2D_SOURCE_DIR})
if(APPLE)
	add_library(UnityPlugin MODULE UnityPlugin.cpp UnityContactListener.h UnityContactListener.cpp)
	target_compile_definitions(UnityPlugin PUBLIC APPLE)
else()
	add_library(UnityPlugin SHARED UnityPlugin.cpp UnityContactListener.h UnityContactListener.cpp)
endif()
target_link_libraries (UnityPlugin Box2D)

set_target_properties(UnityPlugin PROPERTIES
		OUTPUT_NAME "Liquidfun_UnityPlugin"
		CLEAN_DIRECT_OUTPUT 1
		VERSION ${BOX2D_VERSION}
		BUNDLE TRUE
	)
