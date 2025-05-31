set(SOURCES
	orthoconvex.cpp
	parse_tree.cpp
	tree.cpp
	treemap_helpers.cpp
	treemap.cpp
	arrangement_helpers.cpp
	draw_tree.cpp
	turbo_colormap.cpp
	aspect_ratio.cpp
	convex.cpp
)
set(HEADERS
	orthoconvex.h
	parse_tree.h
	tree.h
	treemap_helpers.h
	treemap.h
	turbo_colormap.h
	aspect_ratio.h
	convex.h
	ok_hsv_hsl.h
	draw_tree.h
	arrangement_helpers.h
)

add_library(treemap ${SOURCES})
target_link_libraries(treemap
	PUBLIC core
)

cartocrow_install_module(treemap)
install(FILES ${HEADERS} DESTINATION ${CARTOCROW_INSTALL_DIR}/simplification)
