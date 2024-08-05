set(SOURCES
    treemap_demo.cpp
    parse_csv_to_tree.cpp
    treemap_painting.hpp
    treemap_painting.cpp
)

set(HEADERS
    treemap_painting.h
)

add_executable(treemap_demo ${SOURCES})

target_link_libraries(
        treemap_demo
        PRIVATE
        ${COMMON_CLA_TARGET}
        core
        renderer
        treemap
        CGAL::CGAL
        Qt5::Widgets
)

install(TARGETS treemap_demo DESTINATION ${INSTALL_BINARY_DIR})