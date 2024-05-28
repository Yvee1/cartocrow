set(SOURCES
    treemap_demo.cpp
    tree.cpp
    tree.h
    parse_tree.cpp
    parse_tree.h
)

add_executable(treemap_demo ${SOURCES})

target_link_libraries(
        treemap_demo
        PRIVATE
        ${COMMON_CLA_TARGET}
        core
        renderer
        CGAL::CGAL
        Qt5::Widgets
)

install(TARGETS treemap_demo DESTINATION ${INSTALL_BINARY_DIR})