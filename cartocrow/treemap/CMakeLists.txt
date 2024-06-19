set(SOURCES
	orthoconvex.cpp
	parse_tree.cpp
	rectangle_helpers.cpp
	tree.cpp
	treemap_helpers.cpp
	treemap.cpp
	arrangement_helpers.cpp
	arrangement_helpers.h
	draw_tree.cpp
	draw_tree.h
	turbo_colormap.cpp
	ok_hsv_hsl.h
)
set(HEADERS
	orthoconvex.h
	parse_tree.h
	rectangle_helpers.h
	tree.h
	treemap_helpers.h
	treemap.h
	turbo_colormap.h
	ok_hsv_hsl.h
)

add_library(treemap ${SOURCES})
target_link_libraries(treemap
	PUBLIC core
)

cartocrow_install_module(treemap)
install(FILES ${HEADERS} DESTINATION ${CARTOCROW_INSTALL_DIR}/simplification)
