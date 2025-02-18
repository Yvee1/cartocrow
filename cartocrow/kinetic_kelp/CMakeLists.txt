set(SOURCES
	cat_point.cpp
	parse_input.cpp
	trajectory.cpp
	input.cpp
	input_instance.cpp
	state_geometry.cpp
	state_geometry_painting.cpp
	kinetic_kelp_painting.cpp
	kelp.cpp
	state.h
	route_edges.cpp
	pseudotriangulation.cpp
)

set(HEADERS
	cat_point.h
	parse_input.h
	trajectory.h
	moving_cat_point.h
	kinetic_kelp_painting.h
	input.h
	input_instance.h
	state.h
	state_geometry.h
	kelp_geometry.h
	state_geometry_painting.h
	kelp.h
	route_edges.h
	pseudotriangulation.h
	types.h
	hash.h
)

add_library(kinetic_kelp ${SOURCES})
target_link_libraries(kinetic_kelp
	PUBLIC core
		   circle_segment_helpers
		   renderer
)

cartocrow_install_module(kinetic_kelp)
install(FILES ${HEADERS} DESTINATION ${CARTOCROW_INSTALL_DIR}/kinetic_kelp)
